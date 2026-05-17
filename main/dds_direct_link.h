#pragma once

#include <stdbool.h>
#include <stdint.h>

#define DDS_DIRECT_DEFAULT_PLAYBACK_RATE_HZ 100000U

typedef struct {
    uint32_t sample_rate_hz;
    uint32_t pynq_step_clks;
} dds_direct_playback_rate_t;

bool DdsDirect_Init(void);
uint32_t DdsDirect_DefaultPlaybackRateHz(void);
bool DdsDirect_GetPlaybackRate(uint32_t sample_rate_hz, dds_direct_playback_rate_t *out);
bool DdsDirect_SendWave(const int16_t *samples, uint32_t sample_count, uint32_t sample_rate_hz);
bool DdsDirect_SendSquareTest(void);
bool DdsDirect_SendTriangleTest(void);
bool DdsDirect_SpiEchoTest(void);
