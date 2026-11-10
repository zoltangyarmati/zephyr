/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Zoltan Gyarmati
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for the Analog Devices AD7730 24-bit sigma-delta ADC.
 * Reference: AD7730/AD7730L Data Sheet Rev B.
 *
 * Implements the standard Zephyr ADC driver API (channel_setup/read/
 * read_async via adc_context, same shape as adc_ad4170.c). Functionality
 * with no generic-ADC-API equivalent — sinc filter tuning, burnout
 * detection, self-/system-calibration, and continuous free-running
 * streaming (no generic-API equivalent for "start converting and keep
 * pushing results") — is exposed as a small vendor extension API in
 * <zephyr/drivers/adc/ad7730.h>.
 */

#define DT_DRV_COMPAT adi_ad7730_adc

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/adc/ad7730.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_ad7730, CONFIG_ADC_AD7730_LOG_LEVEL);

/*
 * Register-select values — RS[2:0] field, datasheet Table VIII.
 * RS=0 is dual-purpose: write = Communications Register, read = Status
 * Register.
 */
#define AD7730_REG_STATUS 0x0 /* STATUS (read, 8-bit)  / COMM (write) */
#define AD7730_REG_DATA   0x1 /* Data   (read-only, 16 or 24-bit)     */
#define AD7730_REG_MODE   0x2 /* Mode   (read/write, 16-bit)          */
#define AD7730_REG_FILTER 0x3 /* Filter (read/write, 24-bit)          */
#define AD7730_REG_DAC    0x4 /* DAC    (read/write, 8-bit)           */
#define AD7730_REG_OFFSET 0x5 /* Offset (read/write, 24-bit)          */
#define AD7730_REG_GAIN   0x6 /* Gain   (read/write, 24-bit)          */

/*
 * Communications Register – write-only, first byte of every SPI transaction.
 * Bit layout (datasheet Table VI): [WEN, ZERO, RW1, RW0, ZERO, RS2, RS1, RS0]
 * WEN must be 0 for the write to take effect; bits 6 and 3 must always be 0.
 * RW[1:0] (Table VII): 00=write, 01=single read, 10=start cont. read,
 * 11=stop cont. read.
 */
#define AD7730_COMM_RW_WRITE (0x0u << 4)
#define AD7730_COMM_RW_READ  (0x1u << 4)

#define AD7730_COMM_WRITE(rs) (AD7730_COMM_RW_WRITE | ((rs) & 0x7u))
#define AD7730_COMM_READ(rs)  (AD7730_COMM_RW_READ | ((rs) & 0x7u))

/*
 * Status Register (8-bit, read-only) – datasheet Table IX.
 */
#define AD7730_STATUS_RDY   BIT(7) /* 0 = conversion result ready */
#define AD7730_STATUS_STDY  BIT(6) /* 0 = result is from a fully settled filter */
#define AD7730_STATUS_STBY  BIT(5) /* 1 = part is in standby mode */
#define AD7730_STATUS_NOREF BIT(4) /* 1 = no reference voltage detected */

/*
 * Mode Register (16-bit) – datasheet Table X.
 */
#define AD7730_MODE_MD_MSK GENMASK(15, 13)
#define AD7730_MODE_BU     BIT(12) /* 0=bipolar, 1=unipolar */
#define AD7730_MODE_WL     BIT(8)  /* 0=16-bit Data Register, 1=24-bit */
#define AD7730_MODE_HIREF  BIT(7)
#define AD7730_MODE_RN_MSK GENMASK(5, 4) /* Input range select, Table XII */
#define AD7730_MODE_BO     BIT(2)
#define AD7730_MODE_CH_MSK GENMASK(1, 0) /* Channel pair select, Table XIII */

#define AD7730_MODE_MD(x) FIELD_PREP(AD7730_MODE_MD_MSK, (x))
#define AD7730_MODE_RN(x) FIELD_PREP(AD7730_MODE_RN_MSK, (x))
#define AD7730_MODE_CH(x) FIELD_PREP(AD7730_MODE_CH_MSK, (x))

/*
 * Filter Register (24-bit) – datasheet Table XIV.
 */
#define AD7730_FILTER_SF_MSK GENMASK(23, 12) /* Sinc decimation word */
#define AD7730_FILTER_SKIP   BIT(9)          /* Bypass sinc filter */
#define AD7730_FILTER_FAST   BIT(8)          /* Fast settling mode */
#define AD7730_FILTER_AC     BIT(5)          /* AC excitation mode */
#define AD7730_FILTER_CHP    BIT(4)          /* Chop mode */
#define AD7730_FILTER_DL_MSK GENMASK(3, 0)   /* Chop settling delay */

#define AD7730_FILTER_SF(x) FIELD_PREP(AD7730_FILTER_SF_MSK, (x))
#define AD7730_FILTER_DL(x) FIELD_PREP(AD7730_FILTER_DL_MSK, (x))

/* Power-on defaults (datasheet Table V) */
#define AD7730_MODE_RESET_VAL   0x01B0U
#define AD7730_FILTER_RESET_VAL 0x200010U

/* SPI reset: hold DIN high for ≥32 SCLK cycles */
#define AD7730_RESET_LEN 5U

/* Poll cap for status reads when no drdy-gpios is configured */
#define AD7730_POLL_MAX 100000U

/* AD7730 has four fixed channel pairings (AD7730_CH_* in the dt-bindings
 * header) — channel_id selects the physical input directly, same category
 * of hardware the adc_channel_cfg doc comment calls "dedicated inputs",
 * so CONFIG_ADC_CONFIGURABLE_INPUTS / input_positive / input_negative are
 * not needed.
 */
#define AD7730_NUM_CHANNELS 4U

#define AD7730_CONVERSION_TIMEOUT K_MSEC(CONFIG_ADC_AD7730_CONVERSION_TIMEOUT_MS)

/*
 * Private enums — not part of the public API; mode/range selection is
 * derived from adc_channel_cfg / ad7730_cal_mode instead.
 */
enum ad7730_mode {
	AD7730_MODE_IDLE = 0,
	AD7730_MODE_CONTINUOUS = 1,
	AD7730_MODE_SINGLE = 2,
	AD7730_MODE_STANDBY = 3,
	AD7730_MODE_CAL_INT_ZERO = 4,
	AD7730_MODE_CAL_INT_FULL = 5,
	AD7730_MODE_CAL_SYS_ZERO = 6,
	AD7730_MODE_CAL_SYS_FULL = 7,
};

enum ad7730_input_range {
	AD7730_RANGE_10MV = 0, /* ±10 mV bipolar / 0 to +10 mV unipolar */
	AD7730_RANGE_20MV = 1,
	AD7730_RANGE_40MV = 2,
	AD7730_RANGE_80MV = 3, /* power-on/reset default */
};

/*
 * Driver structs
 */
struct ad7730_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec drdy_gpio; /* optional; falls back to status polling */
	uint16_t vref_mv;
	bool unipolar;
};

struct ad7730_channel_cfg {
	bool configured;
	enum ad7730_input_range range;
};

struct ad7730_data {
	const struct device *dev;
	struct adc_context ctx;

	struct ad7730_channel_cfg channel_cfg[AD7730_NUM_CHANNELS];
	uint16_t enabled_channels;

	int32_t *buffer;
	int32_t *repeat_buffer;

	uint16_t mode_reg;
	uint32_t filter_reg;
	bool burnout_detect;

	struct gpio_callback drdy_gpio_cb;
	struct k_sem drdy_sem;
	struct k_sem acquire_signal;

#if CONFIG_ADC_ASYNC
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_AD7730_ACQUISITION_THREAD_STACK_SIZE);
#endif /* CONFIG_ADC_ASYNC */

	/* Continuous streaming (ad7730_stream_start/stop, vendor extension) */
	struct k_thread stream_thread;
	K_KERNEL_STACK_MEMBER(stream_stack, CONFIG_ADC_AD7730_STREAM_THREAD_STACK_SIZE);
	struct k_sem stream_start_sem;
	ad7730_stream_cb_t stream_cb;
	void *stream_user_data;
	uint8_t stream_channel_id;
	bool stream_active;
};

/*
 * Low-level SPI helpers
 */

/*
 * Compose and send a write transaction: [comm_byte | data bytes].
 * CS is held asserted for the entire transfer as required by the AD7730
 * communications protocol.
 */
static int ad7730_write_reg(const struct device *dev, uint8_t reg, uint32_t value, uint8_t len)
{
	const struct ad7730_config *config = dev->config;
	uint8_t buf[5];
	struct spi_buf tx = {.buf = buf, .len = (size_t)len + 1u};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1u};

	buf[0] = AD7730_COMM_WRITE(reg);
	for (int i = len - 1; i >= 0; i--) {
		buf[len - i] = (value >> (8u * i)) & 0xFFu;
	}

	return spi_write_dt(&config->bus, &tx_set);
}

/*
 * Read a register: send [comm_byte | dummy bytes], receive [dummy | data].
 */
static int ad7730_read_reg(const struct device *dev, uint8_t reg, uint32_t *value, uint8_t len)
{
	const struct ad7730_config *config = dev->config;
	uint8_t tx_buf[5] = {0};
	uint8_t rx_buf[5] = {0};
	struct spi_buf tx = {.buf = tx_buf, .len = (size_t)len + 1u};
	struct spi_buf rx = {.buf = rx_buf, .len = (size_t)len + 1u};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1u};
	const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1u};
	int ret;

	tx_buf[0] = AD7730_COMM_READ(reg);

	ret = spi_transceive_dt(&config->bus, &tx_set, &rx_set);
	if (ret) {
		return ret;
	}

	*value = 0;
	for (uint8_t i = 0; i < len; i++) {
		*value = (*value << 8u) | rx_buf[1u + i];
	}

	return 0;
}

/* Hold DIN high for ≥32 SCLK pulses to force a device reset. */
static int ad7730_reset(const struct device *dev)
{
	const struct ad7730_config *config = dev->config;
	uint8_t buf[AD7730_RESET_LEN];
	struct spi_buf tx = {.buf = buf, .len = sizeof(buf)};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1u};

	memset(buf, 0xFF, sizeof(buf));
	return spi_write_dt(&config->bus, &tx_set);
}

/* Poll the Status Register until RDY=0 or timeout. Fallback path used only
 * when drdy-gpios is not present in devicetree.
 */
static int ad7730_wait_ready(const struct device *dev, k_timeout_t timeout)
{
	int64_t deadline_ms = k_uptime_get() + k_ticks_to_ms_ceil64(timeout.ticks);
	uint32_t val;
	int ret;

	for (uint32_t i = 0; i < AD7730_POLL_MAX; i++) {
		ret = ad7730_read_reg(dev, AD7730_REG_STATUS, &val, 1u);
		if (ret) {
			return ret;
		}
		if (!(val & AD7730_STATUS_RDY)) {
			return 0;
		}
		if (k_uptime_get() >= deadline_ms) {
			return -ETIMEDOUT;
		}
		k_yield();
	}
	return -ETIMEDOUT;
}

/*
 * Register-level helpers
 */

static int ad7730_gain_to_range(enum adc_gain gain, enum ad7730_input_range *range)
{
	/* The AD7730 has no PGA; RN[1:0] instead selects the full-scale input
	 * range, which is the chip's equivalent of a gain setting — halving
	 * the range doubles the effective gain for a fixed external Vref.
	 */
	switch (gain) {
	case ADC_GAIN_1:
		*range = AD7730_RANGE_80MV;
		return 0;
	case ADC_GAIN_2:
		*range = AD7730_RANGE_40MV;
		return 0;
	case ADC_GAIN_4:
		*range = AD7730_RANGE_20MV;
		return 0;
	case ADC_GAIN_8:
		*range = AD7730_RANGE_10MV;
		return 0;
	default:
		return -EINVAL;
	}
}

static uint16_t ad7730_build_mode_reg(const struct device *dev, uint8_t channel_id,
				      enum ad7730_mode mode)
{
	const struct ad7730_config *config = dev->config;
	struct ad7730_data *data = dev->data;
	uint16_t reg = 0;

	reg |= AD7730_MODE_MD(mode);
	if (config->unipolar) {
		reg |= AD7730_MODE_BU;
	}
	if (config->vref_mv == 5000) {
		reg |= AD7730_MODE_HIREF;
	}
	reg |= AD7730_MODE_RN(data->channel_cfg[channel_id].range);
	if (data->burnout_detect) {
		reg |= AD7730_MODE_BO;
	}
	reg |= AD7730_MODE_CH(channel_id);

	return reg;
}

static uint32_t build_filter_reg(const struct ad7730_filter_config *cfg)
{
	uint32_t reg = 0;

	reg |= AD7730_FILTER_SF(cfg->sinc_decimation);
	reg |= AD7730_FILTER_DL(cfg->chop_delay);
	if (cfg->skip) {
		reg |= AD7730_FILTER_SKIP;
	}
	if (cfg->fast) {
		reg |= AD7730_FILTER_FAST;
	}
	if (cfg->ac) {
		reg |= AD7730_FILTER_AC;
	}
	if (cfg->chop) {
		reg |= AD7730_FILTER_CHP;
	}

	return reg;
}

static int ad7730_read_data_reg(const struct device *dev, int32_t *result)
{
	const struct ad7730_config *config = dev->config;
	uint32_t raw;
	int ret;

	ret = ad7730_read_reg(dev, AD7730_REG_DATA, &raw, 3u);
	if (ret) {
		return ret;
	}

	if (config->unipolar) {
		/* Unipolar output coding: straight unsigned 24-bit code,
		 * 0x000000 = zero input, 0xFFFFFF = +Vfs. No sign bit exists.
		 */
		*result = (int32_t)raw;
	} else {
		/* Bipolar output coding is offset binary, not two's complement:
		 * 0x000000 = -Vfs, 0x800000 = zero input, 0xFFFFFF = +Vfs.
		 * Subtracting mid-scale recovers a correctly signed, monotonic
		 * value across the whole code range.
		 */
		*result = (int32_t)raw - 0x800000;
	}
	return 0;
}

/*
 * DRDY handling
 */
static void ad7730_drdy_isr(const struct device *gpio_dev, struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
	ARG_UNUSED(gpio_dev);
	ARG_UNUSED(pins);

	struct ad7730_data *data = CONTAINER_OF(cb, struct ad7730_data, drdy_gpio_cb);

	k_sem_give(&data->drdy_sem);
}

/*
 * Write mode_reg (starting a conversion or calibration sequence) and wait
 * for completion. The DRDY interrupt is armed *before* the register write
 * that triggers the conversion, so the edge can never be missed between
 * arming and triggering.
 */
static int ad7730_trigger_and_wait(const struct device *dev, uint16_t mode_reg, k_timeout_t timeout)
{
	const struct ad7730_config *config = dev->config;
	struct ad7730_data *data = dev->data;
	int ret;

	if (config->drdy_gpio.port) {
		k_sem_reset(&data->drdy_sem);
		ret = gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret) {
			return ret;
		}
	}

	ret = ad7730_write_reg(dev, AD7730_REG_MODE, mode_reg, 2u);
	if (ret == 0) {
		data->mode_reg = mode_reg;

		if (config->drdy_gpio.port) {
			ret = k_sem_take(&data->drdy_sem, timeout);
			ret = (ret == 0) ? 0 : -ETIMEDOUT;
		} else {
			ret = ad7730_wait_ready(dev, timeout);
		}
	}

	if (config->drdy_gpio.port) {
		gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_DISABLE);
	}

	return ret;
}

static int ad7730_perform_channel_read(const struct device *dev, uint8_t channel_id,
				       int32_t *result)
{
	uint16_t mode_reg = ad7730_build_mode_reg(dev, channel_id, AD7730_MODE_SINGLE);
	int ret;

	ret = ad7730_trigger_and_wait(dev, mode_reg, AD7730_CONVERSION_TIMEOUT);
	if (ret) {
		return ret;
	}

	return ad7730_read_data_reg(dev, result);
}

/*
 * adc_context glue
 */
static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ad7730_data *data = CONTAINER_OF(ctx, struct ad7730_data, ctx);

	data->repeat_buffer = data->buffer;
	k_sem_give(&data->acquire_signal);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ad7730_data *data = CONTAINER_OF(ctx, struct ad7730_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int adc_ad7730_validate_sequence(const struct device *dev,
					const struct adc_sequence *sequence)
{
	struct ad7730_data *data = dev->data;
	size_t num_requested_channels;
	size_t necessary;

	if (sequence->resolution != 24) {
		LOG_ERR("Unsupported resolution %u (AD7730 is fixed 24-bit)", sequence->resolution);
		return -EINVAL;
	}

	if (!sequence->channels) {
		LOG_ERR("No channel selected");
		return -EINVAL;
	}

	if (sequence->oversampling) {
		LOG_ERR("Oversampling is not supported");
		return -EINVAL;
	}

	if (sequence->channels & ~data->enabled_channels) {
		LOG_ERR("Requested channel(s) 0x%x not configured via adc_channel_setup()",
			sequence->channels & ~data->enabled_channels);
		return -EINVAL;
	}

	num_requested_channels = POPCOUNT(sequence->channels);
	necessary = num_requested_channels * sizeof(int32_t);
	if (sequence->options) {
		necessary *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < necessary) {
		LOG_ERR("buffer size %u is too small, need %u", sequence->buffer_size, necessary);
		return -ENOMEM;
	}

	return 0;
}

static int adc_ad7730_perform_read(const struct device *dev)
{
	struct ad7730_data *data = dev->data;
	int ret;

	k_sem_take(&data->acquire_signal, K_FOREVER);

	for (uint8_t ch = 0; ch < AD7730_NUM_CHANNELS; ch++) {
		int32_t raw;

		if (!(data->ctx.sequence.channels & BIT(ch))) {
			continue;
		}

		ret = ad7730_perform_channel_read(dev, ch, &raw);
		if (ret) {
			LOG_ERR("channel %u read failed: %d", ch, ret);
			adc_context_complete(&data->ctx, ret);
			return ret;
		}

		*data->buffer++ = raw;
	}

	adc_context_on_sampling_done(&data->ctx, dev);

	return 0;
}

static int adc_ad7730_start_read(const struct device *dev, const struct adc_sequence *sequence,
				 bool wait)
{
	struct ad7730_data *data = dev->data;
	int ret;

	ret = adc_ad7730_validate_sequence(dev, sequence);
	if (ret) {
		return ret;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	if (wait) {
		ret = adc_context_wait_for_completion(&data->ctx);
	}

	return ret;
}

#if CONFIG_ADC_ASYNC
static int adc_ad7730_read_async(const struct device *dev, const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct ad7730_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, true, async);
	ret = adc_ad7730_start_read(dev, sequence, true);
	adc_context_release(&data->ctx, ret);

	return ret;
}

static int adc_ad7730_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ad7730_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);
	ret = adc_ad7730_start_read(dev, sequence, true);
	adc_context_release(&data->ctx, ret);

	return ret;
}

static void adc_ad7730_acquisition_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;

	while (true) {
		adc_ad7730_perform_read(dev);
	}
}
#else
static int adc_ad7730_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ad7730_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);

	ret = adc_ad7730_start_read(dev, sequence, false);
	while (ret == 0 && k_sem_take(&data->ctx.sync, K_NO_WAIT) != 0) {
		ret = adc_ad7730_perform_read(dev);
	}

	adc_context_release(&data->ctx, ret);

	return ret;
}
#endif /* CONFIG_ADC_ASYNC */

/*
 * ADC driver API
 */
static int adc_ad7730_channel_setup(const struct device *dev, const struct adc_channel_cfg *cfg)
{
	struct ad7730_data *data = dev->data;
	enum ad7730_input_range range;
	int ret;

	if (cfg->channel_id >= AD7730_NUM_CHANNELS) {
		LOG_ERR("Invalid channel %u", cfg->channel_id);
		return -EINVAL;
	}

	if (!cfg->differential) {
		LOG_ERR("AD7730 channels are always differential AIN(+)/AIN(-) pairs");
		return -EINVAL;
	}

	if (cfg->reference != ADC_REF_EXTERNAL0) {
		LOG_ERR("AD7730 has no internal reference; use ADC_REF_EXTERNAL0");
		return -EINVAL;
	}

	if (cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Acquisition time is fixed by the filter config "
			"(see ad7730_set_filter), not configurable per channel");
		return -EINVAL;
	}

	ret = ad7730_gain_to_range(cfg->gain, &range);
	if (ret) {
		LOG_ERR("Unsupported gain %u (use ADC_GAIN_1/2/4/8, mapping to "
			"the ±80/40/20/10 mV input ranges)",
			cfg->gain);
		return ret;
	}

	data->channel_cfg[cfg->channel_id].range = range;
	data->channel_cfg[cfg->channel_id].configured = true;
	data->enabled_channels |= BIT(cfg->channel_id);

	return 0;
}

/*
 * Vendor extension API (zephyr/drivers/adc/ad7730.h)
 */
int ad7730_set_filter(const struct device *dev, const struct ad7730_filter_config *cfg)
{
	struct ad7730_data *data = dev->data;
	uint32_t filter_reg;
	int ret;

	if (!cfg) {
		return -EINVAL;
	}

	filter_reg = build_filter_reg(cfg);

	adc_context_lock(&data->ctx, false, NULL);
	ret = ad7730_write_reg(dev, AD7730_REG_FILTER, filter_reg, 3u);
	if (ret == 0) {
		data->filter_reg = filter_reg;
	}
	adc_context_release(&data->ctx, ret);

	return ret;
}

int ad7730_set_burnout_detect(const struct device *dev, bool enable)
{
	struct ad7730_data *data = dev->data;

	adc_context_lock(&data->ctx, false, NULL);
	data->burnout_detect = enable;
	adc_context_release(&data->ctx, 0);

	return 0;
}

int ad7730_calibrate(const struct device *dev, enum ad7730_cal_mode cal_mode, k_timeout_t timeout)
{
	static const enum ad7730_mode mode_map[] = {
		[AD7730_CAL_INT_ZERO] = AD7730_MODE_CAL_INT_ZERO,
		[AD7730_CAL_INT_FULL] = AD7730_MODE_CAL_INT_FULL,
		[AD7730_CAL_SYS_ZERO] = AD7730_MODE_CAL_SYS_ZERO,
		[AD7730_CAL_SYS_FULL] = AD7730_MODE_CAL_SYS_FULL,
	};
	struct ad7730_data *data = dev->data;
	uint16_t mode_reg;
	uint8_t channel_id;
	int ret;

	if (cal_mode > AD7730_CAL_SYS_FULL) {
		return -EINVAL;
	}

	adc_context_lock(&data->ctx, false, NULL);

	if (data->enabled_channels == 0) {
		LOG_ERR("No channel configured; call adc_channel_setup() first");
		ret = -EINVAL;
		goto out;
	}
	/* Calibrates whichever channel pairing is currently selected; with
	 * more than one channel configured, calibrates the lowest-numbered one.
	 */
	channel_id = find_lsb_set(data->enabled_channels) - 1;

	mode_reg = ad7730_build_mode_reg(dev, channel_id, mode_map[cal_mode]);
	ret = ad7730_trigger_and_wait(dev, mode_reg, timeout);

out:
	adc_context_release(&data->ctx, ret);
	return ret;
}

int ad7730_read_status(const struct device *dev, uint8_t *status)
{
	struct ad7730_data *data = dev->data;
	uint32_t val;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);
	ret = ad7730_read_reg(dev, AD7730_REG_STATUS, &val, 1u);
	if (ret == 0) {
		*status = (uint8_t)val;
	}
	adc_context_release(&data->ctx, ret);

	return ret;
}

/*
 * Parked waiting on stream_start_sem until ad7730_stream_start() wakes it;
 * then reads the DATA register off each DRDY edge until ad7730_stream_stop()
 * clears stream_active (which also gives drdy_sem once to unblock the
 * k_sem_take() below so the loop can notice and exit promptly).
 */
static void ad7730_stream_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ad7730_data *data = dev->data;

	while (true) {
		k_sem_take(&data->stream_start_sem, K_FOREVER);

		while (data->stream_active) {
			int32_t raw;
			int ret;

			ret = k_sem_take(&data->drdy_sem, AD7730_CONVERSION_TIMEOUT);
			if (!data->stream_active) {
				break;
			}
			if (ret) {
				LOG_WRN("stream: DRDY timeout");
				continue;
			}

			ret = ad7730_read_data_reg(dev, &raw);
			if (ret) {
				LOG_ERR("stream: data read failed: %d", ret);
				continue;
			}

			data->stream_cb(dev, raw, data->stream_user_data);
		}
	}
}

int ad7730_stream_start(const struct device *dev, uint8_t channel_id, ad7730_stream_cb_t cb,
			void *user_data)
{
	const struct ad7730_config *config = dev->config;
	struct ad7730_data *data = dev->data;
	uint16_t mode_reg;
	int ret;

	if (!cb) {
		return -EINVAL;
	}
	if (!config->drdy_gpio.port) {
		LOG_ERR("Continuous streaming requires drdy-gpios");
		return -ENOTSUP;
	}
	if (channel_id >= AD7730_NUM_CHANNELS) {
		return -EINVAL;
	}

	adc_context_lock(&data->ctx, false, NULL);

	if (!data->channel_cfg[channel_id].configured) {
		LOG_ERR("Channel %u not configured; call adc_channel_setup() first", channel_id);
		adc_context_release(&data->ctx, 0);
		return -EINVAL;
	}

	data->stream_cb = cb;
	data->stream_user_data = user_data;
	data->stream_channel_id = channel_id;
	data->stream_active = true;

	k_sem_reset(&data->drdy_sem);
	ret = gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret == 0) {
		mode_reg = ad7730_build_mode_reg(dev, channel_id, AD7730_MODE_CONTINUOUS);
		ret = ad7730_write_reg(dev, AD7730_REG_MODE, mode_reg, 2u);
		if (ret == 0) {
			data->mode_reg = mode_reg;
		} else {
			gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_DISABLE);
		}
	}

	if (ret) {
		data->stream_active = false;
		adc_context_release(&data->ctx, 0);
		return ret;
	}

	k_sem_give(&data->stream_start_sem);

	/*
	 * adc_context_release() is intentionally NOT called here: the lock
	 * stays held for the whole streaming session (released by
	 * ad7730_stream_stop()), blocking adc_read()/adc_read_async()/the
	 * other vendor calls until streaming stops.
	 */
	return 0;
}

int ad7730_stream_stop(const struct device *dev)
{
	const struct ad7730_config *config = dev->config;
	struct ad7730_data *data = dev->data;
	uint16_t mode_reg;
	int ret;

	if (!data->stream_active) {
		return -EALREADY;
	}

	data->stream_active = false;
	k_sem_give(&data->drdy_sem); /* wake the stream thread out of its wait */

	gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_DISABLE);

	mode_reg = ad7730_build_mode_reg(dev, data->stream_channel_id, AD7730_MODE_IDLE);
	ret = ad7730_write_reg(dev, AD7730_REG_MODE, mode_reg, 2u);
	if (ret == 0) {
		data->mode_reg = mode_reg;
	}

	adc_context_release(&data->ctx, 0);

	return ret;
}

int ad7730_read_all_regs(const struct device *dev, struct ad7730_raw_regs *regs)
{
	struct ad7730_data *data = dev->data;
	uint32_t val;
	int ret;

	if (!regs) {
		return -EINVAL;
	}

	adc_context_lock(&data->ctx, false, NULL);

	ret = ad7730_read_reg(dev, AD7730_REG_MODE, &val, 2u);
	if (ret) {
		goto out;
	}
	regs->mode = (uint16_t)val;

	ret = ad7730_read_reg(dev, AD7730_REG_FILTER, &val, 3u);
	if (ret) {
		goto out;
	}
	regs->filter = val;

	ret = ad7730_read_reg(dev, AD7730_REG_OFFSET, &val, 3u);
	if (ret) {
		goto out;
	}
	regs->offset = val;

	ret = ad7730_read_reg(dev, AD7730_REG_GAIN, &val, 3u);
	if (ret) {
		goto out;
	}
	regs->gain = val;

out:
	adc_context_release(&data->ctx, ret);
	return ret;
}

/*
 * Initialisation
 */
static int ad7730_init(const struct device *dev)
{
	const struct ad7730_config *config = dev->config;
	struct ad7730_data *data = dev->data;
	int ret;

	data->dev = dev;

	k_sem_init(&data->drdy_sem, 0, 1);
	k_sem_init(&data->acquire_signal, 0, 1);
	k_sem_init(&data->stream_start_sem, 0, 1);

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	if (config->drdy_gpio.port) {
		if (!gpio_is_ready_dt(&config->drdy_gpio)) {
			LOG_ERR("DRDY GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT);
		if (ret) {
			return ret;
		}
		gpio_init_callback(&data->drdy_gpio_cb, ad7730_drdy_isr,
				   BIT(config->drdy_gpio.pin));
		ret = gpio_add_callback(config->drdy_gpio.port, &data->drdy_gpio_cb);
		if (ret) {
			return ret;
		}
		/* Interrupt is armed per-conversion in ad7730_trigger_and_wait(). */
	}

	/* Hardware reset */
	ret = ad7730_reset(dev);
	if (ret) {
		LOG_ERR("SPI reset failed: %d", ret);
		return ret;
	}

	/* Allow the serial interface to settle after reset */
	k_sleep(K_MSEC(10));

	/* Load power-on defaults into cache */
	data->mode_reg = AD7730_MODE_RESET_VAL;
	data->filter_reg = AD7730_FILTER_RESET_VAL;

#if CONFIG_ADC_ASYNC
	k_tid_t tid = k_thread_create(&data->thread, data->stack,
				      CONFIG_ADC_AD7730_ACQUISITION_THREAD_STACK_SIZE,
				      adc_ad7730_acquisition_thread, (void *)dev, NULL, NULL,
				      CONFIG_ADC_AD7730_ACQUISITION_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "adc_ad7730");
#endif /* CONFIG_ADC_ASYNC */

	k_tid_t stream_tid = k_thread_create(&data->stream_thread, data->stream_stack,
					     CONFIG_ADC_AD7730_STREAM_THREAD_STACK_SIZE,
					     ad7730_stream_thread, (void *)dev, NULL, NULL,
					     CONFIG_ADC_AD7730_STREAM_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(stream_tid, "adc_ad7730_stream");

	adc_context_unlock_unconditionally(&data->ctx);

	LOG_DBG("AD7730 ready on %s", config->bus.bus->name);
	return 0;
}

static DEVICE_API(adc, ad7730_driver_api) = {
	.channel_setup = adc_ad7730_channel_setup,
	.read = adc_ad7730_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_ad7730_read_async,
#endif
};

/*
 * Device instantiation
 */
/*
 * SPI mode: CPOL=0, CPHA=1 (Mode 1).  POL (datasheet pin 4) is hardware-
 * strapped to GND (POL=0) on this board, confirmed against schematic. Per
 * datasheet "Serial Interface" section: for POL=0, SCLK idles low, the chip
 * latches DIN on the falling edge and drives DOUT on the rising edge —
 * i.e. CPOL=0, CPHA=1.
 */
#define AD7730_SPI_OPERATION (SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8))

#define AD7730_DRDY_GPIO_INIT(inst)                                                                \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, drdy_gpios), \
		(.drdy_gpio = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),))

#define AD7730_DEFINE(inst)                                                                        \
	static const struct ad7730_config ad7730_config_##inst = {                                 \
		.bus = SPI_DT_SPEC_INST_GET(inst, AD7730_SPI_OPERATION),                           \
		.vref_mv = DT_INST_PROP_OR(inst, vref_mv, 2500),                                   \
		.unipolar = DT_INST_PROP_OR(inst, unipolar, false),                                \
		AD7730_DRDY_GPIO_INIT(inst)};                                                      \
	static struct ad7730_data ad7730_data_##inst = {                                           \
		ADC_CONTEXT_INIT_LOCK(ad7730_data_##inst, ctx),                                    \
		ADC_CONTEXT_INIT_TIMER(ad7730_data_##inst, ctx),                                   \
		ADC_CONTEXT_INIT_SYNC(ad7730_data_##inst, ctx),                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, ad7730_init, NULL, &ad7730_data_##inst, &ad7730_config_##inst, \
			      POST_KERNEL, CONFIG_ADC_AD7730_INIT_PRIORITY, &ad7730_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AD7730_DEFINE)
