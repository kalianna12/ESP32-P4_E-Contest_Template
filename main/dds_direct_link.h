#pragma once

#include <stdbool.h>
#include <stdint.h>

bool DdsDirect_Init(void);
bool DdsDirect_SendWave(const int16_t *samples, uint32_t sample_count, uint32_t sample_rate_hz);
bool DdsDirect_SendSquareTest(void);
bool DdsDirect_SendTriangleTest(void);
bool DdsDirect_SpiEchoTest(void);

bool ReconFromCapture_NoFftPassthrough(void);
bool ReconFromCapture_FftCompensate(void);
