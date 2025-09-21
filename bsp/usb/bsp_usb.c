/**
  ******************************************************************************
  * @file    bsp_usb.c
  * @brief   与 USART 风格保持一致的 USB-VCP 单例驱动
  ******************************************************************************
  */
#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include <string.h>

/* 单例句柄 */
static USBInstance *usb_instance = NULL;
static uint8_t      usb_registered = 0;

/* 状态机变量（单例） */
static USBState_e   usb_state = USB_STATE_IDLE;

/* 保存最新一次接收长度，供应用层使用 */
static uint16_t     usb_last_rx_len = 0;

/* 外部 CubeMX 生成句柄 */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef  hpcd_USB_OTG_FS;

/* 正向声明适配器 */
static void RxAdaptor(uint16_t len);
static void TxAdaptor(uint16_t len);

/* -------------------- 对外 API -------------------- */

USBInstance *USBRegister(USB_Init_Config_s *init_config)
{
    if (usb_registered)
    {
        LOGERROR("[bsp_usb] USB already registered!");
        return usb_instance;
    }

    usb_instance = (USBInstance *)malloc(sizeof(USBInstance));
    memset(usb_instance, 0, sizeof(USBInstance));

    /* 基础字段 */
    usb_instance->recv_buff_size = APP_RX_DATA_SIZE;
    usb_instance->usb_handle     = &hUsbDeviceFS;

    /* 三种回调全部存进结构体 */
    usb_instance->module_callback = (usb_module_callback)init_config->rx_cbk; /* 无参 */
    usb_instance->rx_cbk          = RxAdaptor;                               /* 带参 */
    usb_instance->tx_cbk          = TxAdaptor;                               /* 带参 */

    /* 注册给 CDC */
    uint8_t *buf = CDCInitRxbufferNcallback(usb_instance->tx_cbk,
                                            usb_instance->rx_cbk);
    if (buf)
        memcpy(usb_instance->recv_buff, buf, usb_instance->recv_buff_size);

    usb_registered = 1;
    usb_state      = USB_STATE_IDLE;
    LOGINFO("[bsp_usb] USB VCP registered, rx_buff=%p", usb_instance->recv_buff);
    return usb_instance;
}

void USBSend(USBInstance *inst, uint8_t *buf, uint16_t len, USB_TRANSFER_MODE mode)
{
    (void)inst;               /* 单例，暂时忽略 */
    if (!usb_registered)      return;

    if (mode == USB_TRANSFER_DMA)
    {
        uint8_t ret = CDC_Transmit_FS(buf, len);
        if (ret == USBD_OK)  usb_state = USB_STATE_TX;
    }
    /* BLOCKING / IT 暂未实现，留空 */
}

uint8_t USBIsReady(USBInstance *inst)
{
    (void)inst;
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) &&
           (usb_state == USB_STATE_IDLE);
}

void USBReconnect(uint32_t wait_ready_ms)
{
    HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
    DWT_Delay(10);
    HAL_PCD_DevConnect(&hpcd_USB_OTG_FS);

    if (wait_ready_ms)
    {
        uint32_t tick = HAL_GetTick();
        while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED &&
               (HAL_GetTick() - tick) < wait_ready_ms)
            HAL_Delay(1);
    }
    usb_state = USB_STATE_IDLE;
    LOGINFO("[bsp_usb] USB reconnected");
}

uint8_t USBIsConnected(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

/* -------------------- 适配器 -------------------- */

static void RxAdaptor(uint16_t len)
{
    usb_last_rx_len = len;   /* 保存真实长度 */
    if (usb_instance && usb_instance->module_callback)
        ((void (*)(uint16_t))usb_instance->module_callback)(len); /* 强转后调带参版本 */
}

static void TxAdaptor(uint16_t len)
{
    (void)len;
    /* 用户可扩展 */
}

/* -------------------- 弱回调（供 usbd_cdc_if 调用） -------------------- */

__weak void USBD_TxCpltCallback(uint16_t len)
{
    usb_state = USB_STATE_IDLE;
    if (usb_instance && usb_instance->tx_cbk)
        usb_instance->tx_cbk(len);
}

__weak void USBD_RxCpltCallback(uint16_t len)
{
    usb_state = USB_STATE_IDLE;
    if (usb_instance && usb_instance->rx_cbk)
        usb_instance->rx_cbk(len);
}