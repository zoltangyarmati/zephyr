/* SPDX-FileCopyrightText: Copyright (c) 2026 Zoltan Gyarmati
 * SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_AD7730_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_AD7730_H_

/*
 * Vendor extension API for functionality the generic Zephyr ADC API
 * (zephyr/drivers/adc.h) has no equivalent for: sinc-filter tuning, burnout
 * detection, and the chip's self-/system-calibration sequences. Normal
 * sampling goes through adc_channel_setup()/adc_read()/adc_read_async() —
 * see the adi,ad7730-adc.yaml binding for how a channel maps onto one of
 * the four AD7730_CH_* pairings.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calibration sequence selection (datasheet Table XI, MD[2:0]=4..7).
 */
enum ad7730_cal_mode {
	AD7730_CAL_INT_ZERO = 0, /**< Internal zero-scale calibration */
	AD7730_CAL_INT_FULL = 1, /**< Internal full-scale calibration */
	AD7730_CAL_SYS_ZERO = 2, /**< System zero-scale calibration */
	AD7730_CAL_SYS_FULL = 3, /**< System full-scale calibration */
};

/**
 * @brief Digital filter configuration (Filter Register, datasheet Table XIV).
 *
 * Applied to whichever channel is sampled next; takes effect on the next
 * adc_read()/adc_read_async() call.
 */
struct ad7730_filter_config {
	uint16_t sinc_decimation; /**< SF[11:0]: decimation word; controls output data rate */
	uint8_t chop_delay;       /**< DL[3:0]: settling delay applied when CHOP is enabled */
	bool skip;                /**< SKIP: bypass sinc filter section */
	bool fast;                /**< FAST: single-cycle settling mode */
	bool ac;                  /**< AC: enable AC excitation mode */
	bool chop;                /**< CHP: chopping mode — primary recommended mode for best
				   *   drift/noise performance; enabled by default at reset.
				   */
};

/**
 * @brief Configure the digital filter.
 *
 * Unlike mode/range/channel (set via adc_channel_setup()), the filter is not
 * expressible through the generic ADC API and applies across all channels.
 */
int ad7730_set_filter(const struct device *dev, const struct ad7730_filter_config *cfg);

/**
 * @brief Enable or disable the burnout current source for the next conversion.
 *
 * Injects a small constant current into the selected analog input pair to
 * detect an open/burnt-out bridge connection — not expressible through the
 * generic ADC API's channel configuration.
 */
int ad7730_set_burnout_detect(const struct device *dev, bool enable);

/**
 * @brief Run a calibration sequence and wait for completion.
 *
 * Calibrates whichever channel pairing was last configured via
 * adc_channel_setup(). Blocks the calling thread for the duration.
 *
 * @param dev       AD7730 device.
 * @param cal_mode  Calibration sequence to run.
 * @param timeout   Maximum time to wait for calibration to finish.
 */
int ad7730_calibrate(const struct device *dev, enum ad7730_cal_mode cal_mode, k_timeout_t timeout);

/**
 * @brief Read the raw Status Register byte.
 *
 * Bit 7 (RDY=0) indicates data ready. See AD7730 datasheet for all fields.
 */
int ad7730_read_status(const struct device *dev, uint8_t *status);

/**
 * @brief Raw register dump — reads MODE, FILTER, OFFSET, and GAIN in one call.
 *
 * Useful for diagnostic verification that adc_channel_setup()/ad7730_set_filter()
 * actually wrote the expected values to the chip.
 */
struct ad7730_raw_regs {
	uint16_t mode;   /**< Mode Register (16-bit) */
	uint32_t filter; /**< Filter Register (24-bit) */
	uint32_t offset; /**< Offset Register (24-bit) */
	uint32_t gain;   /**< Gain Register (24-bit) */
};

int ad7730_read_all_regs(const struct device *dev, struct ad7730_raw_regs *regs);

/**
 * @brief Streaming sample callback, invoked once per conversion result.
 *
 * Runs on the driver's own stream thread, not interrupt context, but keep
 * it short regardless — it is called back-to-back at the filter's output
 * data rate (see ad7730_set_filter()) for as long as streaming is active.
 */
typedef void (*ad7730_stream_cb_t)(const struct device *dev, int32_t raw, void *user_data);

/**
 * @brief Start continuous (free-running) conversion on one channel.
 *
 * adc_read()/adc_read_async() each re-trigger a discrete conversion and pay
 * the digital filter's settling time on every call. This instead puts the
 * chip into its native continuous-conversion mode: the filter settles once,
 * then @p cb fires for every subsequent result at the filter's output data
 * rate — the quick, high-rate path, e.g. for feeding a moving average.
 *
 * Holds the same lock used by adc_read()/adc_read_async()/the other vendor
 * calls for the whole streaming session: those block until
 * ad7730_stream_stop() is called, since the chip itself cannot be in both
 * continuous and single-conversion mode at once.
 *
 * @param dev         AD7730 device.
 * @param channel_id  Channel previously configured via adc_channel_setup().
 * @param cb          Callback invoked for each result. Must not be NULL.
 * @param user_data   Opaque pointer passed back to @p cb.
 */
int ad7730_stream_start(const struct device *dev, uint8_t channel_id, ad7730_stream_cb_t cb,
			void *user_data);

/**
 * @brief Stop continuous conversion started by ad7730_stream_start().
 *
 * Returns the chip to idle and releases the lock taken by
 * ad7730_stream_start(), unblocking adc_read()/adc_read_async()/the vendor
 * calls again.
 */
int ad7730_stream_stop(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_AD7730_H_ */
