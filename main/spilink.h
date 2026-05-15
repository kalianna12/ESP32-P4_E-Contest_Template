#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t rx_ok;
    uint32_t frame_errors;
    uint32_t timeouts;
    uint32_t dropped_points;
    uint32_t dropped_harmonics;
    uint32_t point_queue_waiting;
    uint32_t point_queue_high_water;
} spilink_stats_t;

void SpiLink_Init(void);
void SpiLink_Task(void);
void SpiLink_UiPump(void);
uint32_t SpiLink_PointQueueWaiting(void);
bool SpiLink_GetStats(spilink_stats_t *out);
void SpiLink_SetPendingCommand(uint32_t cmd, uint32_t arg0, uint32_t arg1);
void SpiLink_SendTextToPynq(const char *text);
void SpiLink_ClearMeasurementCache(void);

#ifdef __cplusplus
}
#endif
