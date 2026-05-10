#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void SpiLink_Init(void);
void SpiLink_Task(void);
void SpiLink_SetPendingCommand(uint32_t cmd, uint32_t arg0, uint32_t arg1);

#ifdef __cplusplus
}
#endif
