/*
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_uhc_otghs

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/usb/uhc.h>

#include "uhc_common.h"
#include <stm32_usb_common.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_stm32, CONFIG_UHC_DRIVER_LOG_LEVEL);

/* Fixed channel assignments for the single-active-transfer model */
#define UHC_STM32_CH_OUT 0U
#define UHC_STM32_CH_IN  1U

enum uhc_stm32_msg_type {
	UHC_STM32_MSG_CONNECT,
	UHC_STM32_MSG_DISCONNECT,
	UHC_STM32_MSG_PORT_ENABLED,
	UHC_STM32_MSG_PORT_DISABLED,
	UHC_STM32_MSG_URB_CHANGE,
};

struct uhc_stm32_msg {
	uint8_t type;
	uint8_t chnum;
	uint8_t urb_state;
	uint8_t _pad;
};

enum uhc_stm32_conn_state {
	UHC_STM32_CONN_IDLE,
	/* Speed-detection reset in progress (we triggered it on connect) */
	UHC_STM32_CONN_INITIAL_RESET,
	/* Stack-triggered enumeration reset in progress */
	UHC_STM32_CONN_STACK_RESET,
};

struct uhc_stm32_data {
	HCD_HandleTypeDef hcd;
	const struct device *dev;
	struct uhc_transfer *active_xfer;
	enum uhc_stm32_conn_state conn_state;
	struct k_thread thread;
	struct k_msgq msgq;
	struct uhc_stm32_msg msgq_buf[CONFIG_UHC_STM32_MAX_QMESSAGES];
};

struct uhc_stm32_config {
	void *base;
	void (*irq_connect)(void);
	uint32_t irqn;
	const struct stm32_pclken *pclken;
	uint8_t num_clocks;
	const struct pinctrl_dev_config *pinctrl;
	const struct stm32_usb_phy *phy;
	uint8_t phy_itface;
};

#define hhcd2data(hhcd) CONTAINER_OF(hhcd, struct uhc_stm32_data, hcd)

/*
 * Returns the 'phy_itface' value for a UHC node.
 *
 * Must use USB_OTG_HS_EMBEDDED_PHY (3) for STM32N6 UTMI+ embedded HS PHY,
 * which is the same value as PCD_PHY_UTMI used by the UDC driver.
 * HCD_PHY_EMBEDDED (2) is only valid for embedded FS PHYs.
 */
#define UHC_STM32_NODE_PHY_ITFACE(usb_node)                                                        \
	COND_CODE_1(DT_NODE_HAS_COMPAT(USB_STM32_PHY(usb_node), usb_ulpi_phy),	\
		(HCD_PHY_ULPI),							\
	(COND_CODE_1(USB_STM32_NODE_PHY_IS_EMBEDDED_HS(usb_node),		\
		(USB_OTG_HS_EMBEDDED_PHY),					\
		(HCD_PHY_EMBEDDED))						\
	))

static uint8_t uhc_stm32_dev_speed(enum usb_device_speed speed)
{
	switch (speed) {
	case USB_SPEED_SPEED_LS:
		return HCD_DEVICE_SPEED_LOW;
	case USB_SPEED_SPEED_HS:
		return HCD_DEVICE_SPEED_HIGH;
	default:
		return HCD_DEVICE_SPEED_FULL;
	}
}

static uint8_t uhc_stm32_ep_type(uint8_t xfer_type)
{
	switch (xfer_type) {
	case USB_EP_TYPE_ISO:
		return EP_TYPE_ISOC;
	case USB_EP_TYPE_BULK:
		return EP_TYPE_BULK;
	case USB_EP_TYPE_INTERRUPT:
		return EP_TYPE_INTR;
	default:
		return EP_TYPE_CTRL;
	}
}

static int uhc_stm32_submit_stage(const struct device *dev, struct uhc_transfer *xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	struct net_buf *buf = xfer->buf;
	uint8_t dev_addr = xfer->udev->addr;
	uint8_t speed = uhc_stm32_dev_speed(xfer->udev->speed);
	uint16_t mps = xfer->mps;
	HAL_StatusTypeDef status = HAL_OK;

	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		switch (xfer->stage) {
		case UHC_CONTROL_STAGE_SETUP: {
			USB_OTG_GlobalTypeDef *USBx = priv->hcd.Instance;
			uint32_t USBx_BASE = (uint32_t)USBx;

			HAL_HCD_HC_Init(&priv->hcd, UHC_STM32_CH_OUT, 0x00, dev_addr, speed,
					EP_TYPE_CTRL, mps);
			status = HAL_HCD_HC_SubmitRequest(&priv->hcd, UHC_STM32_CH_OUT, 0,
							  EP_TYPE_CTRL, 0, xfer->setup_pkt, 8, 0);
			LOG_DBG("SETUP dev=%u speed=%u mps=%u pkt=%02x%02x%02x%02x%02x%02x%02x%02x",
				dev_addr, speed, mps, xfer->setup_pkt[0], xfer->setup_pkt[1],
				xfer->setup_pkt[2], xfer->setup_pkt[3], xfer->setup_pkt[4],
				xfer->setup_pkt[5], xfer->setup_pkt[6], xfer->setup_pkt[7]);
			LOG_DBG("  HCCHAR=0x%08x HCTSIZ=0x%08x HNPTXSTS=0x%08x GUSBCFG=0x%08x",
				USBx_HC(UHC_STM32_CH_OUT)->HCCHAR,
				USBx_HC(UHC_STM32_CH_OUT)->HCTSIZ, USBx->HNPTXSTS, USBx->GUSBCFG);
			break;
		}

		case UHC_CONTROL_STAGE_DATA:
			if (USB_EP_DIR_IS_IN(xfer->ep)) {
				USB_OTG_GlobalTypeDef *USBx = priv->hcd.Instance;
				uint32_t USBx_BASE = (uint32_t)USBx;
				/* wLength from SETUP packet; DMA buffers are aligned to 32 bytes
				 * so tailroom > wLength — must not request more than wLength */
				uint16_t wlen = (uint16_t)xfer->setup_pkt[6] |
						((uint16_t)xfer->setup_pkt[7] << 8);
				uint16_t xfer_len = MIN(net_buf_tailroom(buf), wlen);

				HAL_HCD_HC_Init(&priv->hcd, UHC_STM32_CH_IN, 0x80, dev_addr, speed,
						EP_TYPE_CTRL, mps);
				status = HAL_HCD_HC_SubmitRequest(&priv->hcd, UHC_STM32_CH_IN, 1,
								  EP_TYPE_CTRL, HC_PID_DATA1,
								  net_buf_tail(buf), xfer_len, 0);
				LOG_DBG("DATA_IN dev=%u mps=%u wlen=%u xfer_len=%u", dev_addr, mps,
					(uint32_t)wlen, (uint32_t)xfer_len);
				LOG_DBG("  HCCHAR[1]=0x%08x HCTSIZ[1]=0x%08x",
					USBx_HC(UHC_STM32_CH_IN)->HCCHAR,
					USBx_HC(UHC_STM32_CH_IN)->HCTSIZ);
			} else {
				HAL_HCD_HC_Init(&priv->hcd, UHC_STM32_CH_OUT, 0x00, dev_addr, speed,
						EP_TYPE_CTRL, mps);
				status = HAL_HCD_HC_SubmitRequest(&priv->hcd, UHC_STM32_CH_OUT, 0,
								  EP_TYPE_CTRL, HC_PID_DATA1,
								  buf->data, buf->len, 0);
			}
			break;

		case UHC_CONTROL_STAGE_STATUS:
			if (USB_EP_DIR_IS_IN(xfer->ep)) {
				/* Zero-length OUT to ack IN data; token=1+length=0 makes HAL set
				 * toggle_out=1 → DATA1 PID as required by USB spec */
				HAL_HCD_HC_Init(&priv->hcd, UHC_STM32_CH_OUT, 0x00, dev_addr, speed,
						EP_TYPE_CTRL, mps);
				status = HAL_HCD_HC_SubmitRequest(&priv->hcd, UHC_STM32_CH_OUT, 0,
								  EP_TYPE_CTRL, 1, NULL,
								  0, 0);
			} else {
				/* Zero-length IN to ack OUT data */
				HAL_HCD_HC_Init(&priv->hcd, UHC_STM32_CH_IN, 0x80, dev_addr, speed,
						EP_TYPE_CTRL, mps);
				status = HAL_HCD_HC_SubmitRequest(&priv->hcd, UHC_STM32_CH_IN, 1,
								  EP_TYPE_CTRL, HC_PID_DATA1, NULL,
								  0, 0);
			}
			break;

		default:
			return -EINVAL;
		}
	} else {
		uint8_t ep_idx = USB_EP_GET_IDX(xfer->ep);
		uint8_t ep_type = uhc_stm32_ep_type(xfer->type);

		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			HAL_HCD_HC_Init(&priv->hcd, UHC_STM32_CH_IN, ep_idx | 0x80, dev_addr, speed,
					ep_type, mps);
			status = HAL_HCD_HC_SubmitRequest(&priv->hcd, UHC_STM32_CH_IN, 1, ep_type,
							  1, net_buf_tail(buf),
							  net_buf_tailroom(buf), 0);
		} else {
			HAL_HCD_HC_Init(&priv->hcd, UHC_STM32_CH_OUT, ep_idx, dev_addr, speed,
					ep_type, mps);
			status = HAL_HCD_HC_SubmitRequest(&priv->hcd, UHC_STM32_CH_OUT, 0, ep_type,
							  1, buf->data, buf->len, 0);
		}
	}

	return (status == HAL_OK) ? 0 : -EIO;
}

static int uhc_stm32_schedule(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (priv->active_xfer != NULL) {
		return 0;
	}

	priv->active_xfer = uhc_xfer_get_next(dev);
	if (priv->active_xfer == NULL) {
		return 0;
	}

	return uhc_stm32_submit_stage(dev, priv->active_xfer);
}

static int uhc_stm32_ctrl_done(const struct device *dev, struct uhc_transfer *xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	uint32_t actual;

	switch (xfer->stage) {
	case UHC_CONTROL_STAGE_SETUP:
		LOG_DBG("SETUP done, advancing to %s", xfer->buf ? "DATA" : "STATUS");
		if (xfer->buf != NULL) {
			xfer->stage = UHC_CONTROL_STAGE_DATA;
		} else if (!xfer->no_status) {
			xfer->stage = UHC_CONTROL_STAGE_STATUS;
		} else {
			uhc_xfer_return(dev, xfer, 0);
			priv->active_xfer = NULL;
			return uhc_stm32_schedule(dev);
		}
		return uhc_stm32_submit_stage(dev, xfer);

	case UHC_CONTROL_STAGE_DATA:
		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			actual = HAL_HCD_HC_GetXferCount(&priv->hcd, UHC_STM32_CH_IN);
			net_buf_add(xfer->buf, actual);
		}
		if (!xfer->no_status) {
			xfer->stage = UHC_CONTROL_STAGE_STATUS;
			return uhc_stm32_submit_stage(dev, xfer);
		}
		uhc_xfer_return(dev, xfer, 0);
		priv->active_xfer = NULL;
		return uhc_stm32_schedule(dev);

	case UHC_CONTROL_STAGE_STATUS:
		uhc_xfer_return(dev, xfer, 0);
		priv->active_xfer = NULL;
		return uhc_stm32_schedule(dev);

	default:
		return -EINVAL;
	}
}

static int uhc_stm32_bulk_done(const struct device *dev, struct uhc_transfer *xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	uint32_t actual;

	if (USB_EP_DIR_IS_IN(xfer->ep)) {
		actual = HAL_HCD_HC_GetXferCount(&priv->hcd, UHC_STM32_CH_IN);
		net_buf_add(xfer->buf, actual);
	}

	uhc_xfer_return(dev, xfer, 0);
	priv->active_xfer = NULL;

	return uhc_stm32_schedule(dev);
}

static void uhc_stm32_handle_urb(const struct device *dev, uint8_t chnum, uint8_t urb_state)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	struct uhc_transfer *xfer = priv->active_xfer;
	int err = 0;

	if (xfer == NULL) {
		return;
	}

	switch ((HCD_URBStateTypeDef)urb_state) {
	case URB_DONE:
		if (USB_EP_GET_IDX(xfer->ep) == 0) {
			err = uhc_stm32_ctrl_done(dev, xfer);
		} else {
			err = uhc_stm32_bulk_done(dev, xfer);
		}
		break;
	case URB_NOTREADY:
		//		LOG_WRN("NOTREADY ch %u stage %u ep 0x%02x errcnt %u",
		//			chnum, xfer->stage, xfer->ep,
		//			priv->hcd.hc[chnum].ErrCnt);
		if (chnum == UHC_STM32_CH_OUT && priv->hcd.hc[chnum].ErrCnt == 0) {
			/* OUT NAK: HAL halted but did not re-activate; re-submit */
			err = uhc_stm32_submit_stage(dev, xfer);
		}
		/* IN NAK/XACTERR and OUT XACTERR: HAL already re-activated channel */
		break;
	case URB_STALL: {
		USB_OTG_GlobalTypeDef *USBx = priv->hcd.Instance;
		uint32_t USBx_BASE = (uint32_t)USBx;

		LOG_WRN("STALL on ch %u stage %u ep 0x%02x HCTSIZ=0x%08x HCCHAR=0x%08x", chnum,
			xfer->stage, xfer->ep, USBx_HC(chnum)->HCTSIZ, USBx_HC(chnum)->HCCHAR);
		uhc_xfer_return(dev, xfer, -EPIPE);
		priv->active_xfer = NULL;
		err = uhc_stm32_schedule(dev);
		break;
	}
	case URB_ERROR:
	default:
		LOG_WRN("ERROR ch %u stage %u ep 0x%02x hc_state %u errcnt %u", chnum, xfer->stage,
			xfer->ep, priv->hcd.hc[chnum].state, priv->hcd.hc[chnum].ErrCnt);
		uhc_xfer_return(dev, xfer, -EIO);
		priv->active_xfer = NULL;
		err = uhc_stm32_schedule(dev);
		break;
	}

	if (err) {
		uhc_submit_event(dev, UHC_EVT_ERROR, err);
	}
}

static void uhc_stm32_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	struct uhc_stm32_msg msg;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_msgq_get(&priv->msgq, &msg, K_FOREVER);

		uhc_lock_internal(dev, K_FOREVER);

		switch (msg.type) {
		case UHC_STM32_MSG_CONNECT:
			/* Speed is not yet known — HS devices only chirp during reset.
			 * Trigger a speed-detection reset ourselves; the true speed will
			 * be read from HPRT PRTSPD in the PORT_ENABLED handler. */
			LOG_INF("Device connected, starting speed-detection reset");
			priv->conn_state = UHC_STM32_CONN_INITIAL_RESET;
			HAL_HCD_ResetPort(&priv->hcd);
			break;

		case UHC_STM32_MSG_DISCONNECT:
			priv->conn_state = UHC_STM32_CONN_IDLE;
			if (priv->active_xfer != NULL) {
				uhc_xfer_return(dev, priv->active_xfer, -ECONNRESET);
				priv->active_xfer = NULL;
			}
			uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);
			break;

		case UHC_STM32_MSG_PORT_ENABLED: {
			USB_OTG_GlobalTypeDef *USBx = priv->hcd.Instance;
			uint32_t USBx_BASE = (uint32_t)USBx;
			uint32_t hprt = *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE);
			uint32_t prtspd = (hprt >> 17) & 3u;

			LOG_DBG("Port enabled: HCFG=0x%08x HFIR=0x%08x HPRT=0x%08x GCCFG=0x%08x",
				USBx_HOST->HCFG, USBx_HOST->HFIR, hprt, USBx->GCCFG);
			LOG_DBG("  PPWR=%u PRTSPD=%u PENA=%u GUSBCFG=0x%08x", (hprt >> 12) & 1u,
				prtspd, (hprt >> 2) & 1u, USBx->GUSBCFG);
			LOG_DBG("  HCFG unchanged: 0x%08x", USBx_HOST->HCFG);

			if (priv->conn_state == UHC_STM32_CONN_INITIAL_RESET) {
				/* Speed-detection reset complete — now PRTSPD is authoritative.
				 * Report the real speed to the stack; it will call uhc_bus_reset
				 * for the enumeration reset, which produces a second PORT_ENABLED. */
				priv->conn_state = UHC_STM32_CONN_STACK_RESET;
				LOG_INF("Speed detection done: PRTSPD=%u (0=HS,1=FS,2=LS)", prtspd);
				if (prtspd == HPRT0_PRTSPD_HIGH_SPEED) {
					uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_HS, 0);
				} else if (prtspd == HPRT0_PRTSPD_LOW_SPEED) {
					uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_LS, 0);
				} else {
					uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_FS, 0);
				}
			} else {
				/* Stack-triggered enumeration reset complete — start transfers. */
				priv->conn_state = UHC_STM32_CONN_IDLE;
				uhc_submit_event(dev, UHC_EVT_RESETED, 0);
				uhc_stm32_schedule(dev);
			}
			break;
		}

		case UHC_STM32_MSG_PORT_DISABLED:
			uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);
			break;

		case UHC_STM32_MSG_URB_CHANGE:
			uhc_stm32_handle_urb(dev, msg.chnum, msg.urb_state);
			break;

		default:
			break;
		}

		uhc_unlock_internal(dev);
	}
}

/* HAL callbacks — called from HAL_HCD_IRQHandler, i.e. interrupt context */

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
	struct uhc_stm32_data *priv = hhcd2data(hhcd);
	const struct uhc_stm32_msg msg = {.type = UHC_STM32_MSG_CONNECT};

	k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
	struct uhc_stm32_data *priv = hhcd2data(hhcd);
	const struct uhc_stm32_msg msg = {.type = UHC_STM32_MSG_DISCONNECT};

	k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
}

void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd)
{
	struct uhc_stm32_data *priv = hhcd2data(hhcd);
	const struct uhc_stm32_msg msg = {.type = UHC_STM32_MSG_PORT_ENABLED};
	uint32_t speed = HAL_HCD_GetCurrentSpeed(hhcd);

	LOG_DBG("PortEnabled ISR: HPRT speed=%u (0=HS,1=FS,2=LS)", speed);
	k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
}

void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd)
{
	struct uhc_stm32_data *priv = hhcd2data(hhcd);
	const struct uhc_stm32_msg msg = {.type = UHC_STM32_MSG_PORT_DISABLED};

	k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
}

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
					 HCD_URBStateTypeDef urb_state)
{
	struct uhc_stm32_data *priv = hhcd2data(hhcd);

	/* HAL auto-re-activates IN channels after NAK — no action needed from us.
	 * Drop these in the ISR to prevent msgq flooding that would cause OUT NOTREADY
	 * messages (which do need processing) to be lost. */
	if (chnum == UHC_STM32_CH_IN && urb_state == URB_NOTREADY) {
		return;
	}

	const struct uhc_stm32_msg msg = {
		.type = UHC_STM32_MSG_URB_CHANGE,
		.chnum = chnum,
		.urb_state = (uint8_t)urb_state,
	};

	k_msgq_put(&priv->msgq, &msg, K_NO_WAIT);
}

static int uhc_stm32_clock_enable(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int err;

	if (cfg->num_clocks > 1) {
		if (clock_control_configure(clk, (clock_control_subsys_t)&cfg->pclken[1], NULL) !=
		    0) {
			LOG_ERR("Failed to select USB clock source");
			return -EIO;
		}
	}

	if (clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]) != 0) {
		LOG_ERR("Failed to enable USB clock");
		return -EIO;
	}

	if (cfg->phy != NULL) {
		err = cfg->phy->enable(cfg->phy);
		if (err != 0) {
			LOG_ERR("Failed to enable USB PHY: %d", err);
			return err;
		}
	}

	return 0;
}

static int uhc_stm32_clock_disable(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int err;

	if (cfg->phy != NULL) {
		err = cfg->phy->disable(cfg->phy);
		if (err != 0) {
			LOG_ERR("Failed to disable USB PHY: %d", err);
			return err;
		}
	}

	if (clock_control_off(clk, (clock_control_subsys_t)&cfg->pclken[0]) != 0) {
		LOG_ERR("Failed to disable USB clock");
		return -EIO;
	}

	return 0;
}

static int uhc_stm32_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int uhc_stm32_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

static int uhc_stm32_init(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *cfg = dev->config;
	HAL_StatusTypeDef status;
	int err;

	err = stm32_usb_pwr_enable();
	if (err != 0) {
		LOG_ERR("Failed to enable USB power: %d", err);
		return err;
	}

	err = uhc_stm32_clock_enable(dev);
	if (err != 0) {
		return err;
	}

	memset(&priv->hcd, 0, sizeof(priv->hcd));
	priv->hcd.Instance = cfg->base;
	priv->hcd.Init.Host_channels = 16;
	priv->hcd.Init.speed = HCD_SPEED_HIGH;
	priv->hcd.Init.dma_enable = DISABLE;
	priv->hcd.Init.phy_itface = cfg->phy_itface;
	priv->hcd.Init.Sof_enable = DISABLE;
	priv->hcd.Init.low_power_enable = DISABLE;
	priv->hcd.Init.vbus_sensing_enable = DISABLE;
	priv->hcd.Init.use_external_vbus = ENABLE;

	status = HAL_HCD_Init(&priv->hcd);
	if (status != HAL_OK) {
		LOG_ERR("HAL_HCD_Init failed (%d)", (int)status);
		return -EIO;
	}

	return 0;
}

static int uhc_stm32_enable(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *cfg = dev->config;
	HAL_StatusTypeDef status;

	status = HAL_HCD_Start(&priv->hcd);
	if (status != HAL_OK) {
		LOG_ERR("HAL_HCD_Start failed (%d)", (int)status);
		return -EIO;
	}

	irq_enable(cfg->irqn);

	return 0;
}

static int uhc_stm32_disable(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	irq_disable(cfg->irqn);

	HAL_HCD_Stop(&priv->hcd);

	return 0;
}

static int uhc_stm32_shutdown(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	HAL_HCD_DeInit(&priv->hcd);

	uhc_stm32_clock_disable(dev);

	stm32_usb_pwr_disable();

	if (irq_is_enabled(cfg->irqn)) {
		irq_disable(cfg->irqn);
	}

	return 0;
}

static int uhc_stm32_bus_reset(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	return (HAL_HCD_ResetPort(&priv->hcd) == HAL_OK) ? 0 : -EIO;
}

static int uhc_stm32_sof_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* SOF generation is started during uhc_stm32_enable via HAL_HCD_Start */
	return 0;
}

static int uhc_stm32_bus_suspend(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	USB_OTG_GlobalTypeDef *USBx = priv->hcd.Instance;
	uint32_t hprt0;

	hprt0 = *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE);
	hprt0 &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET | USB_OTG_HPRT_PENCHNG |
		   USB_OTG_HPRT_POCCHNG);
	hprt0 |= USB_OTG_HPRT_PSUSP;
	*(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE) = hprt0;

	return 0;
}

static int uhc_stm32_bus_resume(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	USB_OTG_GlobalTypeDef *USBx = priv->hcd.Instance;
	uint32_t hprt0;

	hprt0 = *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE);
	hprt0 &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET | USB_OTG_HPRT_PENCHNG |
		   USB_OTG_HPRT_POCCHNG);
	hprt0 |= USB_OTG_HPRT_PRES;
	*(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE) = hprt0;

	k_msleep(20);

	/* Clear resume signaling */
	hprt0 = *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE);
	hprt0 &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET | USB_OTG_HPRT_PENCHNG |
		   USB_OTG_HPRT_POCCHNG | USB_OTG_HPRT_PRES);
	*(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE) = hprt0;

	return 0;
}

static int uhc_stm32_ep_enqueue(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	int err;

	err = uhc_xfer_append(dev, xfer);
	if (err != 0) {
		return err;
	}

	if (priv->active_xfer == NULL) {
		err = uhc_stm32_schedule(dev);
	}

	return err;
}

static int uhc_stm32_ep_dequeue(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	unsigned int key;

	key = irq_lock();
	if (priv->active_xfer == xfer) {
		priv->active_xfer = NULL;
	}
	irq_unlock(key);

	uhc_xfer_return(dev, xfer, -ECONNRESET);

	return 0;
}

static const struct uhc_api uhc_stm32_api = {
	.lock = uhc_stm32_lock,
	.unlock = uhc_stm32_unlock,
	.init = uhc_stm32_init,
	.enable = uhc_stm32_enable,
	.disable = uhc_stm32_disable,
	.shutdown = uhc_stm32_shutdown,
	.bus_reset = uhc_stm32_bus_reset,
	.sof_enable = uhc_stm32_sof_enable,
	.bus_suspend = uhc_stm32_bus_suspend,
	.bus_resume = uhc_stm32_bus_resume,
	.ep_enqueue = uhc_stm32_ep_enqueue,
	.ep_dequeue = uhc_stm32_ep_dequeue,
};

/*
 * Device ordinals (DT_DEP_ORD) are used instead of instance numbers because
 * this driver can instantiate from multiple compatible strings in a single
 * build, and instance numbers are only unique per compatible.
 */
#define UHC_STM32_DEFINE(node_id, ord)                                                             \
	K_THREAD_STACK_DEFINE(CONCAT(uhc, ord, _thr_stk), CONFIG_UHC_STM32_STACK_SIZE);            \
                                                                                                   \
	static struct uhc_stm32_data CONCAT(uhc, ord, _priv);                                      \
                                                                                                   \
	static struct uhc_data CONCAT(uhc, ord, _data) = {                                         \
		.mutex = Z_MUTEX_INITIALIZER(CONCAT(uhc, ord, _data).mutex),                       \
		.priv = &CONCAT(uhc, ord, _priv),                                                  \
	};                                                                                         \
                                                                                                   \
	static void CONCAT(uhc, ord, _irq_connect)(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQ_BY_NAME(node_id, otghs, irq),                                   \
			    DT_IRQ_BY_NAME(node_id, otghs, priority), HAL_HCD_IRQHandler,          \
			    &CONCAT(uhc, ord, _priv).hcd, 0);                                      \
	}                                                                                          \
                                                                                                   \
	PINCTRL_DT_DEFINE(node_id);                                                                \
                                                                                                   \
	static const struct stm32_pclken CONCAT(uhc, ord, _pclken)[] = STM32_DT_CLOCKS(node_id);   \
                                                                                                   \
	static const struct uhc_stm32_config CONCAT(uhc, ord, _cfg) = {                            \
		.base = (void *)DT_REG_ADDR(node_id),                                              \
		.irq_connect = &CONCAT(uhc, ord, _irq_connect),                                    \
		.irqn = DT_IRQ_BY_NAME(node_id, otghs, irq),                                       \
		.pclken = CONCAT(uhc, ord, _pclken),                                               \
		.num_clocks = DT_NUM_CLOCKS(node_id),                                              \
		.pinctrl = PINCTRL_DT_DEV_CONFIG_GET(node_id),                                     \
		.phy = USB_STM32_PHY_PSEUDODEV_GET_OR_NULL(node_id),                               \
		.phy_itface = UHC_STM32_NODE_PHY_ITFACE(node_id),                                  \
	};                                                                                         \
                                                                                                   \
	static int CONCAT(uhc, ord, _driver_init)(const struct device *dev)                        \
	{                                                                                          \
		struct uhc_stm32_data *priv = uhc_get_private(dev);                                \
		struct uhc_data *data = dev->data;                                                 \
		const struct uhc_stm32_config *cfg = dev->config;                                  \
		int err;                                                                           \
                                                                                                   \
		priv->dev = dev;                                                                   \
                                                                                                   \
		k_msgq_init(&priv->msgq, (char *)priv->msgq_buf, sizeof(struct uhc_stm32_msg),     \
			    CONFIG_UHC_STM32_MAX_QMESSAGES);                                       \
                                                                                                   \
		k_mutex_init(&data->mutex);                                                        \
                                                                                                   \
		k_thread_create(&priv->thread, CONCAT(uhc, ord, _thr_stk),                         \
				K_THREAD_STACK_SIZEOF(CONCAT(uhc, ord, _thr_stk)),                 \
				uhc_stm32_thread, (void *)dev, NULL, NULL,                         \
				K_PRIO_COOP(CONFIG_UHC_STM32_THREAD_PRIORITY), K_ESSENTIAL,        \
				K_NO_WAIT);                                                        \
		k_thread_name_set(&priv->thread, dev->name);                                       \
                                                                                                   \
		cfg->irq_connect();                                                                \
                                                                                                   \
		err = pinctrl_apply_state(cfg->pinctrl, PINCTRL_STATE_DEFAULT);                    \
		if (err < 0 && err != -ENOENT) {                                                   \
			LOG_ERR("pinctrl setup failed (%d)", err);                                 \
			return err;                                                                \
		}                                                                                  \
                                                                                                   \
		return 0;                                                                          \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_DEFINE(node_id, CONCAT(uhc, ord, _driver_init), NULL, &CONCAT(uhc, ord, _data),  \
			 &CONCAT(uhc, ord, _cfg), POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			 &uhc_stm32_api);

#define UHC_STM32_FOREACH_DEFINE(node_id) UHC_STM32_DEFINE(node_id, DT_DEP_ORD(node_id))

DT_FOREACH_STATUS_OKAY(st_stm32_uhc_otghs, UHC_STM32_FOREACH_DEFINE)
DT_FOREACH_STATUS_OKAY(st_stm32n6_uhc_otghs, UHC_STM32_FOREACH_DEFINE)
