#include "bsp_log.h"
#include "string.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>


int JS_RTT_Channel = 1;        //

void BSPLogInit()
{
    SEGGER_RTT_Init();
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}

Val_t *JScopeInit(Va_conf config)
{
    Val_t *val = (Val_t*)malloc(sizeof(Val_t));
    memset(val, 0, sizeof(Val_t));
    val->Val = config.Val;
    SEGGER_RTT_ConfigUpBuffer(JS_RTT_Channel,
                              "JScope_I4",
                              &val->JS_RTT_UpBuffer[0],
                              sizeof(val->JS_RTT_UpBuffer),
                              SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
    return val;
}

void JScope_Updata(Val_t *val)
{
    SEGGER_RTT_Write(JS_RTT_Channel, val->Val, sizeof(&val->Val));
}