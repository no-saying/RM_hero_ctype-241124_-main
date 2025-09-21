/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CANInstance *_instance)
{
    uint16_t *rxbuff;
    SuperCap_Msg_s *Msg;
    rxbuff = (uint16_t*)_instance->rx_buff;
    Msg = &super_cap_instance->cap_msg;
    Msg->vol = rxbuff[0] / 100.f;
    Msg->cvol = rxbuff[1] / 100.f;
    Msg->current = rxbuff[2] / 100.f;
    Msg->power = rxbuff[3] / 100.f;
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));
    
    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins,1);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}