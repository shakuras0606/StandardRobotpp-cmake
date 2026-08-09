/*
 * VOFA+ Protocol Driver - Header
 * https://github.com/jelin-sh/VOFA-Protocol-Driver
 */

#ifndef VOFA_H
#define VOFA_H

#include <stdbool.h>
#include <stdint.h>

#ifndef VOFA_BUFFER_SIZE
#define VOFA_BUFFER_SIZE 256
#endif

#ifndef VOFA_CMD_TAIL
#define VOFA_CMD_TAIL \
    {                 \
        0xAF, 0xFA    \
    }
#endif

typedef struct
{
    uint8_t buffer[VOFA_BUFFER_SIZE];
    uint8_t *rp;
    uint8_t *wp;
    bool overflow;
} Vofa_FIFOTypeDef;

typedef enum
{
    VOFA_MODE_SKIP,
    VOFA_MODE_BLOCK_IF_FIFO_FULL
} Vofa_ModeTypeDef;

typedef struct
{
    uint8_t txBuffer[VOFA_BUFFER_SIZE];
    Vofa_FIFOTypeDef rxBuffer;
    Vofa_ModeTypeDef mode;
} Vofa_HandleTypedef;

void Vofa_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode);

/* ---- 发送 ---- */
void Vofa_SendData(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t num);
void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num);
void Vofa_Printf(Vofa_HandleTypedef *handle, const char *format, ...);

/* ---- 接收 ---- */
void Vofa_ReceiveData(Vofa_HandleTypedef *handle);
uint16_t Vofa_ReadCmd(Vofa_HandleTypedef *handle, uint8_t *buffer, uint16_t bufferLen);
uint16_t Vofa_ReadLine(Vofa_HandleTypedef *handle, uint8_t *buffer, uint16_t bufferLen);
uint16_t Vofa_ReadData(Vofa_HandleTypedef *handle, uint8_t *buffer, uint16_t bufferLen);

/* ---- 用户必须实现的回调 ---- */
extern void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length);
extern uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle);

#endif
