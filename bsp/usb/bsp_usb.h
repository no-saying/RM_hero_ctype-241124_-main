#ifndef BSP_USB_H
#define BSP_USB_H
#pragma once
#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* 与 usart 保持一致 */
#define USART_RXBUFF_LIMIT 256
typedef void (*usb_module_callback)(void);
typedef void (*USBCallback)(uint16_t len);   /* CDC 需要带长度 */

/* USB 状态枚举 */
typedef enum
{
    USB_STATE_IDLE,
    USB_STATE_TX,
    USB_STATE_RX
} USBState_e;

/* 发送模式枚举，与 usart 同名 */
typedef enum
{
    USB_TRANSFER_NONE = 0,
    USB_TRANSFER_BLOCKING,
    USB_TRANSFER_IT,
    USB_TRANSFER_DMA,
} USB_TRANSFER_MODE;

/* 与 USARTInstance 一一对应的实例结构体 */
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; /* 静态缓冲区 */
    uint8_t recv_buff_size;                /* 一包数据最大长度 */
    USBD_HandleTypeDef *usb_handle;        /* 对应 usb 句柄 */
    usb_module_callback module_callback;   /* 应用层无参解析回调 */

    /* 下面两条供 CDC 内部使用，上层可忽略 */
    USBCallback rx_cbk;                    /* 接收完成（带长度） */
    USBCallback tx_cbk;                    /* 发送完成（带长度） */
} USBInstance;

/* 初始化配置结构体：与 USART 侧风格一致 */
typedef struct
{
    usb_module_callback rx_cbk;   /* 应用层无参解析回调 */
    usb_module_callback tx_cbk;   /* 应用层无参发送回调（可 NULL） */
} USB_Init_Config_s;

/* 对外 API：与 USART 完全同名同风格 */
USBInstance *USBRegister(USB_Init_Config_s *init_config);
void         USBSend(USBInstance *inst, uint8_t *buf, uint16_t len, USB_TRANSFER_MODE mode);
uint8_t      USBIsReady(USBInstance *inst);
void         USBReconnect(uint32_t wait_ready_ms);
uint8_t      USBIsConnected(void);

/* 弱回调，供 usbd_cdc_if 调用 */
void USBD_TxCpltCallback(uint16_t len);
void USBD_RxCpltCallback(uint16_t len);
#endif