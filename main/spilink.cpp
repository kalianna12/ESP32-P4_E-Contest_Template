#include "spilink.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "ui/test_screen.h"

#ifndef ENABLE_SPI_STRING_TEST
#define ENABLE_SPI_STRING_TEST 0
#endif

namespace {

constexpr char TAG[] = "FreqRespSpiLink";

constexpr spi_host_device_t SPI_RX_HOST = SPI3_HOST;
constexpr gpio_num_t SPI_RX_MOSI = GPIO_NUM_21;
constexpr gpio_num_t SPI_RX_MISO = GPIO_NUM_22;
constexpr gpio_num_t SPI_RX_SCLK = GPIO_NUM_23;
constexpr gpio_num_t SPI_RX_CS = GPIO_NUM_3;
constexpr TickType_t SPI_RX_TIMEOUT_TICKS = pdMS_TO_TICKS(200);

constexpr uint8_t kFrameMagic0 = 0xA5;
constexpr uint8_t kFrameMagic1 = 0x5A;
constexpr uint8_t kFrameTypeFreqRespStatus = 0x10;
constexpr uint8_t kFrameTypeFreqRespPoint = 0x13;
constexpr uint8_t kFrameTypeAdvStatus = 0x14;
constexpr uint8_t kFrameTypeAdvWaveChunk = 0x15;
constexpr uint8_t kFrameTypeAdvHarmonic = 0x16;
constexpr uint8_t kFrameTypeCommand = 0x80;
constexpr uint8_t kFrameTypePynqToEspText = 0xE1;
constexpr uint8_t kFrameTypeEspToPynqText = 0xE2;
constexpr uint8_t kPayloadLen = 112;
constexpr uint8_t kCommandPayloadLen = 16;
constexpr size_t kTextMaxLen = 104;

constexpr size_t kFrameHeaderLen = 4;
constexpr size_t kFrameLen = kFrameHeaderLen + kPayloadLen + 1;
constexpr size_t kSpiTransferLen = 128;

constexpr size_t kDmaAlign = 64;
constexpr size_t kRxBufferLen = 128;
constexpr size_t kTxBufferLen = 128;
constexpr size_t kCommandQueueLen = 16;
constexpr UBaseType_t kStatusQueueLen = 1;
constexpr UBaseType_t kPointQueueLen = 256;
constexpr UBaseType_t kAdvStatusQueueLen = 4;
constexpr UBaseType_t kAdvWaveQueueLen = 64;
constexpr UBaseType_t kAdvHarmonicQueueLen = 64;
constexpr size_t kMaxUiPointsPerPump = 128;
constexpr size_t kMaxAdvWavesPerPump = 0;
constexpr size_t kMaxAdvHarmonicsPerPump = 16;

static_assert((kRxBufferLen % 64) == 0, "SPI RX buffer length must be a 64-byte multiple");
static_assert((kTxBufferLen % 64) == 0, "SPI TX buffer length must be a 64-byte multiple");
static_assert(kFrameLen == 117, "Status logical frame is header + 112-byte payload + checksum");
static_assert(kRxBufferLen == kSpiTransferLen, "SPI RX transaction must stay fixed at 128 bytes");
static_assert(kTxBufferLen == kSpiTransferLen, "SPI TX transaction must stay fixed at 128 bytes");

typedef struct {
    uint32_t cmd;
    uint32_t arg0;
    uint32_t arg1;
} pending_command_t;

typedef struct {
    uint32_t packets;
    uint32_t frame_errors;
    uint32_t timeouts;
    uint32_t progress_permille;
    uint32_t current_freq_hz;
    uint32_t point_index;
    uint32_t total_points;
    int32_t vin_mv;
    int32_t vout_mv;
    int32_t gain_x1000;
    int32_t phase_deg_x10;
    uint32_t flags;
    bool phase_valid;
} point_queue_item_t;

static_assert(sizeof(point_queue_item_t) <= 64, "Point queue item must stay small");

bool spi_ready = false;
uint8_t *rx_buffer = nullptr;
uint8_t *tx_buffer = nullptr;

uint32_t ok_count = 0;
uint32_t err_count = 0;
uint32_t timeout_count = 0;
uint32_t dropped_point_count = 0;
uint32_t dropped_harmonic_count = 0;
uint32_t point_queue_high_water = 0;
uint32_t last_point_index = 0;
uint32_t last_freq_hz = 0;
TickType_t last_log_tick = 0;
TickType_t last_invalid_log_tick = 0;
freqresp_ui_status_t g_last_measurement_status = {};
bool g_have_measurement_status = false;
bool g_have_phase_history = false;
int32_t g_prev_phase_deg_x10 = 0;
QueueHandle_t g_status_queue = nullptr;
QueueHandle_t g_point_queue = nullptr;
QueueHandle_t g_adv_status_queue = nullptr;
QueueHandle_t g_adv_wave_queue = nullptr;
QueueHandle_t g_adv_harmonic_queue = nullptr;

portMUX_TYPE g_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_measurement_lock = portMUX_INITIALIZER_UNLOCKED;
uint32_t g_cmd_seq = 0;
pending_command_t g_cmd_queue[kCommandQueueLen] = {};
size_t g_cmd_head = 0;
size_t g_cmd_tail = 0;
size_t g_cmd_count = 0;

#if ENABLE_SPI_STRING_TEST
portMUX_TYPE g_text_lock = portMUX_INITIALIZER_UNLOCKED;
uint32_t g_rx_text_seq = 0;
uint32_t g_tx_text_seq = 0;
char g_tx_text[kTextMaxLen + 1] = {};
#endif

uint8_t Checksum(const uint8_t *frame, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= frame[i];
    }
    return checksum;
}

uint32_t GetU32(const uint8_t *buffer, size_t offset)
{
    return static_cast<uint32_t>(buffer[offset + 0]) |
           (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
           (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
           (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}

int32_t GetI32(const uint8_t *buffer, size_t offset)
{
    return static_cast<int32_t>(GetU32(buffer, offset));
}

int16_t GetI16(const uint8_t *buffer, size_t offset)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(buffer[offset + 0]) |
        (static_cast<uint16_t>(buffer[offset + 1]) << 8)
    );
}

int32_t RawVppCodeToMv(uint32_t code)
{
    return static_cast<int32_t>((static_cast<uint64_t>(code) * 10000ULL) / 4095ULL);
}

void FillPointQueueItem(const freqresp_ui_status_t *status, point_queue_item_t *item)
{
    if (status == nullptr || item == nullptr) {
        return;
    }

    item->packets = status->packets;
    item->frame_errors = status->frame_errors;
    item->timeouts = status->timeouts;
    item->progress_permille = status->progress_permille;
    item->current_freq_hz = status->current_freq_hz;
    item->point_index = status->point_index;
    item->total_points = status->total_points;
    item->vin_mv = status->vin_mv;
    item->vout_mv = status->vout_mv;
    item->gain_x1000 = status->gain_x1000;
    item->phase_deg_x10 = status->phase_deg_x10;
    item->flags = status->flags;
    item->phase_valid = status->phase_valid;
}

void FillStatusFromPointQueueItem(const point_queue_item_t *item, freqresp_ui_status_t *status)
{
    if (item == nullptr || status == nullptr) {
        return;
    }

    *status = {};
    status->packets = item->packets;
    status->frame_errors = item->frame_errors;
    status->timeouts = item->timeouts;
    status->link_ok = 1U;
    status->state = static_cast<uint8_t>(
        (item->total_points != 0U && item->point_index >= item->total_points) ?
        FREQRESP_STATE_DONE : FREQRESP_STATE_SCANNING
    );
    status->mode = MODE_SWEEP;
    status->filter_type = FILTER_TYPE_UNKNOWN;
    status->progress_permille = item->progress_permille;
    status->current_freq_hz = item->current_freq_hz;
    status->point_index = item->point_index;
    status->total_points = item->total_points;
    status->vin_mv = item->vin_mv;
    status->vout_mv = item->vout_mv;
    status->gain_x1000 = item->gain_x1000;
    status->phase_deg_x10 = item->phase_deg_x10;
    status->flags = item->flags;
    status->has_measurement = true;
    status->phase_valid = item->phase_valid;
}

void PutU32(uint8_t *buffer, size_t offset, uint32_t value)
{
    buffer[offset + 0] = static_cast<uint8_t>((value >> 0) & 0xFFU);
    buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
    buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
}

size_t TextLen104(const char *text)
{
    size_t len = 0;
    while (len < kTextMaxLen && text[len] != '\0') {
        ++len;
    }
    return len;
}

bool PopPendingCommand(pending_command_t *out)
{
    bool has_cmd = false;

    portENTER_CRITICAL(&g_cmd_lock);
    if (g_cmd_count > 0U) {
        *out = g_cmd_queue[g_cmd_head];
        g_cmd_head = (g_cmd_head + 1U) % kCommandQueueLen;
        --g_cmd_count;
        has_cmd = true;
    }
    portEXIT_CRITICAL(&g_cmd_lock);

    return has_cmd;
}

bool ParseFreqRespStatusFrame(const uint8_t *frame, size_t len, freqresp_ui_status_t *out)
{
    if (frame == nullptr || out == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeFreqRespStatus || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];
    if (rx_checksum != expected_checksum) {
        return false;
    }

    size_t o = kFrameHeaderLen;

    out->state = static_cast<uint8_t>(GetU32(frame, o));       o += 4;
    out->mode = static_cast<uint8_t>(GetU32(frame, o));        o += 4;
    out->filter_type = static_cast<uint8_t>(GetU32(frame, o)); o += 4;
    out->link_ok = static_cast<uint8_t>(GetU32(frame, o));     o += 4;

    out->progress_permille = GetU32(frame, o);                 o += 4;

    out->start_freq_hz = GetU32(frame, o);                     o += 4;
    out->stop_freq_hz = GetU32(frame, o);                      o += 4;
    out->step_freq_hz = GetU32(frame, o);                      o += 4;
    out->single_freq_hz = GetU32(frame, o);                    o += 4;

    out->current_freq_hz = GetU32(frame, o);                   o += 4;
    out->point_index = GetU32(frame, o);                       o += 4;
    out->total_points = GetU32(frame, o);                      o += 4;

    out->vin_mv = GetI32(frame, o);                            o += 4;
    out->vout_mv = GetI32(frame, o);                           o += 4;
    out->gain_x1000 = GetI32(frame, o);                        o += 4;
    out->theory_gain_x1000 = GetI32(frame, o);                 o += 4;
    out->error_x10 = GetI32(frame, o);                         o += 4;
    out->phase_deg_x10 = GetI32(frame, o);                     o += 4;

    out->cutoff_freq_hz = GetU32(frame, o);                    o += 4;

    return true;
}

bool ParseFreqRespPointFrame(const uint8_t *frame, size_t len, freqresp_ui_status_t *out)
{
    if (frame == nullptr || out == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeFreqRespPoint || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];
    if (rx_checksum != expected_checksum) {
        return false;
    }

    // 0x13 point result, 128-byte SPI transaction:
    // byte 0=A5, 1=5A, 2=13, 3=112
    // byte 4 seq, 8 point_index, 12 total_points, 16 freq_req_hz, 20 freq_actual_hz
    // byte 24 amp_a_code, 28 amp_b_code, 32 raw_a_vpp_code, 36 raw_b_vpp_code
    // byte 40 gain_x1000, 44 phase_deg_x10, 48 flags, 116 checksum xor(frame[0..115])
    // PYNQADC currently sends point_index as 1-based and leaves gain_x1000 as 0.
    const uint32_t point_index = GetU32(frame, 8);
    const uint32_t total_points = GetU32(frame, 12);
    const uint32_t freq_actual_hz = GetU32(frame, 20);
    const uint32_t raw_a_vpp_code = GetU32(frame, 32) & 0x0FFFU;
    const uint32_t raw_b_vpp_code = GetU32(frame, 36) & 0x0FFFU;
    const int32_t frame_gain_x1000 = GetI32(frame, 40);
    int32_t phase_deg_x10 = GetI32(frame, 44);
    const uint32_t flags = GetU32(frame, 48);
    const bool phase_level_ok = (raw_a_vpp_code >= 80U) && (raw_b_vpp_code >= 40U);

    portENTER_CRITICAL(&g_measurement_lock);
    if (g_have_phase_history) {
        while ((phase_deg_x10 - g_prev_phase_deg_x10) > 1800) {
            phase_deg_x10 -= 3600;
        }
        while ((phase_deg_x10 - g_prev_phase_deg_x10) < -1800) {
            phase_deg_x10 += 3600;
        }
    }

    bool phase_valid = phase_level_ok && ((flags & FREQ_POINT_FLAG_UNSTABLE) == 0U) &&
                       ((flags & 0x00000200U) != 0U);
    if (phase_valid && g_have_phase_history) {
        const int32_t phase_delta = phase_deg_x10 - g_prev_phase_deg_x10;
        if (phase_delta > 600 || phase_delta < -600) {
            phase_valid = false;
        }
    }

    if (phase_valid) {
        g_prev_phase_deg_x10 = phase_deg_x10;
        g_have_phase_history = true;
    }
    portEXIT_CRITICAL(&g_measurement_lock);

    out->state = static_cast<uint8_t>(
        (total_points != 0U && point_index >= total_points) ?
        FREQRESP_STATE_DONE : FREQRESP_STATE_SCANNING
    );
    out->mode = MODE_SWEEP;
    out->filter_type = FILTER_TYPE_UNKNOWN;
    out->link_ok = 1U;
    out->progress_permille = (total_points != 0U) ? ((point_index * 1000U) / total_points) : 0U;
    if (out->progress_permille > 1000U) {
        out->progress_permille = 1000U;
    }
    out->current_freq_hz = freq_actual_hz;
    out->point_index = point_index;
    out->total_points = total_points;
    out->vin_mv = RawVppCodeToMv(raw_a_vpp_code);
    out->vout_mv = RawVppCodeToMv(raw_b_vpp_code);
    out->gain_x1000 = (frame_gain_x1000 > 0) ?
        frame_gain_x1000 :
        ((raw_a_vpp_code != 0U) ?
        static_cast<int32_t>((static_cast<uint64_t>(raw_b_vpp_code) * 1000ULL) / raw_a_vpp_code) :
        0);
    out->theory_gain_x1000 = 0;
    out->error_x10 = 0;
    out->phase_deg_x10 = phase_deg_x10;
    out->flags = flags;
    out->cutoff_freq_hz = 0;
    out->has_measurement = true;
    out->phase_valid = phase_valid;

    return true;
}

bool ParseAdvStatusFrame(const uint8_t *frame, size_t len, adv_status_t *out)
{
    if (frame == nullptr || out == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeAdvStatus || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];
    if (rx_checksum != expected_checksum) {
        return false;
    }

    out->seq = GetU32(frame, 4);
    out->adv_state = GetU32(frame, 8);
    out->error_code = GetU32(frame, 12);
    out->flags = GetU32(frame, 16);
    out->sample_rate_hz = GetU32(frame, 20);
    out->total_sample_count = GetU32(frame, 24);
    out->capture_done_count = GetU32(frame, 28);
    out->recon_done_count = GetU32(frame, 32);
    out->fft_overflow_count = GetU32(frame, 36);
    out->ifft_overflow_count = GetU32(frame, 40);
    out->tlast_missing_count = GetU32(frame, 44);
    out->tlast_unexpected_count = GetU32(frame, 48);
    return true;
}

bool ParseAdvWaveChunkFrame(const uint8_t *frame, size_t len, adc_waveform_chunk_t *out)
{
    if (frame == nullptr || out == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeAdvWaveChunk || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];
    if (rx_checksum != expected_checksum) {
        return false;
    }

    out->seq = GetU32(frame, 4);
    out->wave_type = GetU32(frame, 8);
    out->chunk_index = GetU32(frame, 12);
    out->chunk_count = GetU32(frame, 16);
    out->sample_rate_hz = GetU32(frame, 20);
    out->dds_freq_hz = 0;
    out->total_sample_count = GetU32(frame, 24);
    out->start_sample_index = GetU32(frame, 28);
    out->min_mv = GetI32(frame, 32);
    out->max_mv = GetI32(frame, 36);
    out->mean_mv = GetI32(frame, 40);
    out->vpp_mv = GetI32(frame, 44);
    out->flags = GetU32(frame, 48);
    for (size_t i = 0; i < 30U; ++i) {
        out->samples[i] = GetI16(frame, 52 + i * 2U);
    }
    return true;
}

bool ParseAdvHarmonicFrame(const uint8_t *frame, size_t len, adv_harmonic_t *out)
{
    if (frame == nullptr || out == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeAdvHarmonic || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];
    if (rx_checksum != expected_checksum) {
        return false;
    }

    out->seq = GetU32(frame, 4);
    out->index = GetU32(frame, 8);
    out->freq_hz = GetU32(frame, 12);
    out->amp_mv = GetI32(frame, 16);
    out->phase_deg_x10 = GetI32(frame, 20);
    out->flags = GetU32(frame, 24);
    return true;
}

#if ENABLE_SPI_STRING_TEST
bool ParseTextFrame(const uint8_t *frame, size_t len, uint8_t expected_type, uint32_t *seq, char *text, size_t text_len)
{
    if (frame == nullptr || seq == nullptr || text == nullptr || text_len == 0U || len < kSpiTransferLen) {
        return false;
    }

    text[0] = '\0';

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != expected_type || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];
    if (rx_checksum != expected_checksum) {
        return false;
    }

    *seq = GetU32(frame, 4);
    uint32_t rx_text_len = GetU32(frame, 8);
    if (rx_text_len > kTextMaxLen) {
        rx_text_len = kTextMaxLen;
    }
    if (rx_text_len >= text_len) {
        rx_text_len = static_cast<uint32_t>(text_len - 1U);
    }

    std::memcpy(text, &frame[12], rx_text_len);
    text[rx_text_len] = '\0';
    return true;
}

void BuildTextFrame(uint8_t *frame)
{
    std::memset(frame, 0x00, kTxBufferLen);

    char text[kTextMaxLen + 1] = {};
    uint32_t seq = 0;

    portENTER_CRITICAL(&g_text_lock);
    seq = g_tx_text_seq;
    std::memcpy(text, g_tx_text, sizeof(text));
    portEXIT_CRITICAL(&g_text_lock);

    frame[0] = kFrameMagic0;
    frame[1] = kFrameMagic1;
    frame[2] = kFrameTypeEspToPynqText;
    frame[3] = kPayloadLen;

    const uint32_t len = static_cast<uint32_t>(TextLen104(text));
    PutU32(frame, 4, seq);
    PutU32(frame, 8, len);
    if (len > 0U) {
        std::memcpy(&frame[12], text, len);
    }

    frame[kFrameHeaderLen + kPayloadLen] = Checksum(frame, kFrameHeaderLen + kPayloadLen);
}

#endif

void PrepareTxBuffer()
{
    std::memset(tx_buffer, 0x00, kTxBufferLen);

#if ENABLE_SPI_STRING_TEST
    portENTER_CRITICAL(&g_text_lock);
    const bool has_text_frame = (g_tx_text_seq != 0U);
    portEXIT_CRITICAL(&g_text_lock);
    if (has_text_frame) {
        BuildTextFrame(tx_buffer);
        return;
    }
#endif

    pending_command_t pending = {};
    const bool has_cmd = PopPendingCommand(&pending);

    tx_buffer[0] = kFrameMagic0;
    tx_buffer[1] = kFrameMagic1;
    tx_buffer[2] = kFrameTypeCommand;
    tx_buffer[3] = kCommandPayloadLen;

    uint32_t seq = 0;
    if (has_cmd) {
        portENTER_CRITICAL(&g_cmd_lock);
        ++g_cmd_seq;
        if (g_cmd_seq == 0U) {
            ++g_cmd_seq;
        }
        seq = g_cmd_seq;
        portEXIT_CRITICAL(&g_cmd_lock);
    }

    size_t o = kFrameHeaderLen;
    PutU32(tx_buffer, o, seq);          o += 4;
    PutU32(tx_buffer, o, pending.cmd);  o += 4;
    PutU32(tx_buffer, o, pending.arg0); o += 4;
    PutU32(tx_buffer, o, pending.arg1); o += 4;

    tx_buffer[kFrameHeaderLen + kCommandPayloadLen] =
        Checksum(tx_buffer, kFrameHeaderLen + kCommandPayloadLen);
}

void FreeBuffers()
{
    if (rx_buffer != nullptr) {
        heap_caps_free(rx_buffer);
        rx_buffer = nullptr;
    }

    if (tx_buffer != nullptr) {
        heap_caps_free(tx_buffer);
        tx_buffer = nullptr;
    }
}

void FreeQueues()
{
    if (g_status_queue != nullptr) {
        vQueueDelete(g_status_queue);
        g_status_queue = nullptr;
    }

    if (g_point_queue != nullptr) {
        vQueueDelete(g_point_queue);
        g_point_queue = nullptr;
    }

    if (g_adv_status_queue != nullptr) {
        vQueueDelete(g_adv_status_queue);
        g_adv_status_queue = nullptr;
    }

    if (g_adv_wave_queue != nullptr) {
        vQueueDelete(g_adv_wave_queue);
        g_adv_wave_queue = nullptr;
    }

    if (g_adv_harmonic_queue != nullptr) {
        vQueueDelete(g_adv_harmonic_queue);
        g_adv_harmonic_queue = nullptr;
    }
}

void MaybeUpdatePointQueueHighWater()
{
    if (g_point_queue == nullptr) {
        return;
    }

    const UBaseType_t waiting = uxQueueMessagesWaiting(g_point_queue);
    if (static_cast<uint32_t>(waiting) > point_queue_high_water) {
        point_queue_high_water = static_cast<uint32_t>(waiting);
    }
}

void MaybeLogSpiStats()
{
    const TickType_t now = xTaskGetTickCount();
    if (last_log_tick != 0 &&
        (now - last_log_tick) < pdMS_TO_TICKS(1000)) {
        return;
    }

    last_log_tick = now;
    const UBaseType_t point_waiting =
        (g_point_queue != nullptr) ? uxQueueMessagesWaiting(g_point_queue) : 0;

    ESP_LOGI(TAG,
             "SPI stats: ok=%lu err=%lu timeout=%lu dropped_points=%lu dropped_harmonics=%lu point_queue_waiting=%u point_queue_high_water=%lu last_point_index=%lu last_freq_hz=%lu",
             static_cast<unsigned long>(ok_count),
             static_cast<unsigned long>(err_count),
             static_cast<unsigned long>(timeout_count),
             static_cast<unsigned long>(dropped_point_count),
             static_cast<unsigned long>(dropped_harmonic_count),
             static_cast<unsigned>(point_waiting),
             static_cast<unsigned long>(point_queue_high_water),
             static_cast<unsigned long>(last_point_index),
             static_cast<unsigned long>(last_freq_hz));
}

void MaybeLogInvalidFrame(size_t rx_bits)
{
    const TickType_t now = xTaskGetTickCount();
    if (last_invalid_log_tick != 0 &&
        (now - last_invalid_log_tick) < pdMS_TO_TICKS(1000)) {
        return;
    }

    last_invalid_log_tick = now;
    ESP_LOGW(TAG,
             "Invalid freq response frame: bits=%u data=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X err=%lu",
             static_cast<unsigned>(rx_bits),
             rx_buffer[0],
             rx_buffer[1],
             rx_buffer[2],
             rx_buffer[3],
             rx_buffer[4],
             rx_buffer[5],
             rx_buffer[6],
             rx_buffer[7],
             rx_buffer[8],
             rx_buffer[9],
             rx_buffer[10],
             rx_buffer[11],
             rx_buffer[12],
             rx_buffer[13],
             rx_buffer[14],
             rx_buffer[15],
             static_cast<unsigned long>(err_count));
}

}  // namespace

void SpiLink_Init(void)
{
    if (spi_ready) {
        return;
    }

    rx_buffer = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(
            kDmaAlign,
            kRxBufferLen,
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
        )
    );

    tx_buffer = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(
            kDmaAlign,
            kTxBufferLen,
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
        )
    );

    if (rx_buffer == nullptr || tx_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate SPI DMA buffers");
        FreeBuffers();
        return;
    }

    g_status_queue = xQueueCreate(kStatusQueueLen, sizeof(freqresp_ui_status_t));
    g_point_queue = xQueueCreate(kPointQueueLen, sizeof(point_queue_item_t));
    g_adv_status_queue = xQueueCreate(kAdvStatusQueueLen, sizeof(adv_status_t));
    g_adv_wave_queue = xQueueCreate(kAdvWaveQueueLen, sizeof(adc_waveform_chunk_t));
    g_adv_harmonic_queue = xQueueCreate(kAdvHarmonicQueueLen, sizeof(adv_harmonic_t));
    if (g_status_queue == nullptr ||
        g_point_queue == nullptr ||
        g_adv_status_queue == nullptr ||
        g_adv_wave_queue == nullptr ||
        g_adv_harmonic_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create SPI UI queues");
        FreeQueues();
        FreeBuffers();
        return;
    }

    std::memset(rx_buffer, 0x00, kRxBufferLen);
    std::memset(tx_buffer, 0x00, kTxBufferLen);

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SPI_RX_MOSI;
    bus_cfg.miso_io_num = SPI_RX_MISO;
    bus_cfg.sclk_io_num = SPI_RX_SCLK;
    bus_cfg.quadwp_io_num = GPIO_NUM_NC;
    bus_cfg.quadhd_io_num = GPIO_NUM_NC;
    bus_cfg.max_transfer_sz = kRxBufferLen;

    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode = 0;
    slave_cfg.spics_io_num = SPI_RX_CS;
    slave_cfg.queue_size = 3;
    slave_cfg.flags = 0;

    gpio_set_pull_mode(SPI_RX_CS, GPIO_PULLUP_ONLY);

    const esp_err_t err = spi_slave_initialize(
        SPI_RX_HOST,
        &bus_cfg,
        &slave_cfg,
        SPI_DMA_CH_AUTO
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_initialize failed: %s", esp_err_to_name(err));
        FreeQueues();
        FreeBuffers();
        return;
    }

    ok_count = 0;
    err_count = 0;
    timeout_count = 0;
    dropped_point_count = 0;
    dropped_harmonic_count = 0;
    point_queue_high_water = 0;
    last_point_index = 0;
    last_freq_hz = 0;
    last_log_tick = 0;
    last_invalid_log_tick = 0;
    SpiLink_ClearMeasurementCache();
    spi_ready = true;

    ESP_LOGI(TAG,
             "SPI slave ready: host=SPI3 MOSI=%d MISO=%d SCLK=%d CS=%d rx_len=%u",
             static_cast<int>(SPI_RX_MOSI),
             static_cast<int>(SPI_RX_MISO),
             static_cast<int>(SPI_RX_SCLK),
             static_cast<int>(SPI_RX_CS),
             static_cast<unsigned>(kRxBufferLen));
}

void SpiLink_SetPendingCommand(uint32_t cmd, uint32_t arg0, uint32_t arg1)
{
    portENTER_CRITICAL(&g_cmd_lock);

    if (g_cmd_count >= kCommandQueueLen) {
        g_cmd_head = (g_cmd_head + 1U) % kCommandQueueLen;
        --g_cmd_count;
    }

    g_cmd_queue[g_cmd_tail].cmd = cmd;
    g_cmd_queue[g_cmd_tail].arg0 = arg0;
    g_cmd_queue[g_cmd_tail].arg1 = arg1;
    g_cmd_tail = (g_cmd_tail + 1U) % kCommandQueueLen;
    ++g_cmd_count;

    portEXIT_CRITICAL(&g_cmd_lock);
}

void SpiLink_ClearMeasurementCache(void)
{
    portENTER_CRITICAL(&g_measurement_lock);
    g_last_measurement_status = {};
    g_have_measurement_status = false;
    g_have_phase_history = false;
    g_prev_phase_deg_x10 = 0;
    portEXIT_CRITICAL(&g_measurement_lock);

    if (g_status_queue != nullptr) {
        xQueueReset(g_status_queue);
    }
    if (g_point_queue != nullptr) {
        xQueueReset(g_point_queue);
    }
    if (g_adv_status_queue != nullptr) {
        xQueueReset(g_adv_status_queue);
    }
    if (g_adv_wave_queue != nullptr) {
        xQueueReset(g_adv_wave_queue);
    }
    if (g_adv_harmonic_queue != nullptr) {
        xQueueReset(g_adv_harmonic_queue);
    }
    dropped_point_count = 0;
    dropped_harmonic_count = 0;
    point_queue_high_water = 0;
    last_point_index = 0;
    last_freq_hz = 0;
}

void SpiLink_UiPump(void)
{
    if (!spi_ready || g_status_queue == nullptr || g_point_queue == nullptr) {
        return;
    }

    freqresp_ui_status_t status = {};
    if (xQueueReceive(g_status_queue, &status, 0) == pdTRUE) {
        test_screen_update_measurement(&status);
    }

    for (size_t i = 0; i < kMaxUiPointsPerPump; ++i) {
        point_queue_item_t point_item = {};
        if (xQueueReceive(g_point_queue, &point_item, 0) != pdTRUE) {
            break;
        }
        freqresp_ui_status_t point = {};
        FillStatusFromPointQueueItem(&point_item, &point);
        test_screen_update_measurement(&point);
    }

    for (size_t i = 0; i < 2U; ++i) {
        adv_status_t adv = {};
        if (g_adv_status_queue == nullptr ||
            xQueueReceive(g_adv_status_queue, &adv, 0) != pdTRUE) {
            break;
        }
        test_screen_update_adv_status(&adv);
    }

    for (size_t i = 0; i < kMaxAdvWavesPerPump; ++i) {
        adc_waveform_chunk_t chunk = {};
        if (g_adv_wave_queue == nullptr ||
            xQueueReceive(g_adv_wave_queue, &chunk, 0) != pdTRUE) {
            break;
        }
        test_screen_update_adc_waveform_chunk(&chunk);
    }

    for (size_t i = 0; i < kMaxAdvHarmonicsPerPump; ++i) {
        adv_harmonic_t harmonic = {};
        if (g_adv_harmonic_queue == nullptr ||
            xQueueReceive(g_adv_harmonic_queue, &harmonic, 0) != pdTRUE) {
            break;
        }
        test_screen_update_adv_harmonic(&harmonic);
    }
}

uint32_t SpiLink_PointQueueWaiting(void)
{
    if (g_point_queue == nullptr) {
        return 0U;
    }

    return static_cast<uint32_t>(uxQueueMessagesWaiting(g_point_queue));
}

bool SpiLink_GetStats(spilink_stats_t *out)
{
    if (out == nullptr) {
        return false;
    }

    out->rx_ok = ok_count;
    out->frame_errors = err_count;
    out->timeouts = timeout_count;
    out->dropped_points = dropped_point_count;
    out->dropped_harmonics = dropped_harmonic_count;
    out->point_queue_waiting =
        (g_point_queue != nullptr) ? static_cast<uint32_t>(uxQueueMessagesWaiting(g_point_queue)) : 0U;
    out->point_queue_high_water = point_queue_high_water;
    return true;
}

void SpiLink_SendTextToPynq(const char *text)
{
#if ENABLE_SPI_STRING_TEST
    if (text == nullptr) {
        return;
    }

    portENTER_CRITICAL(&g_text_lock);
    ++g_tx_text_seq;
    if (g_tx_text_seq == 0U) {
        ++g_tx_text_seq;
    }
    std::memset(g_tx_text, 0x00, sizeof(g_tx_text));
    std::strncpy(g_tx_text, text, kTextMaxLen);
    portEXIT_CRITICAL(&g_text_lock);
#else
    (void)text;
#endif
}

void SpiLink_Task(void)
{
    if (!spi_ready || rx_buffer == nullptr || tx_buffer == nullptr) {
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    std::memset(rx_buffer, 0x00, kRxBufferLen);
    PrepareTxBuffer();

    spi_slave_transaction_t trans = {};
    trans.length = kRxBufferLen * 8;
    trans.rx_buffer = rx_buffer;
    trans.tx_buffer = tx_buffer;

    const esp_err_t err = spi_slave_transmit(
        SPI_RX_HOST,
        &trans,
        SPI_RX_TIMEOUT_TICKS
    );

    if (err == ESP_ERR_TIMEOUT) {
        ++timeout_count;
        MaybeLogSpiStats();
        return;
    }

    if (err != ESP_OK) {
        ++err_count;
        ESP_LOGW(TAG,
                 "spi_slave_transmit failed: %s err=%lu",
                 esp_err_to_name(err),
                 static_cast<unsigned long>(err_count));
        vTaskDelay(pdMS_TO_TICKS(10));
        MaybeLogSpiStats();
        return;
    }

    const size_t rx_bits = static_cast<size_t>(trans.trans_len);
    if (rx_bits != (kSpiTransferLen * 8U)) {
        ++err_count;
        ESP_LOGW(TAG,
                 "Unexpected SPI transaction length: bits=%u expected=%u err=%lu",
                 static_cast<unsigned>(rx_bits),
                 static_cast<unsigned>(kSpiTransferLen * 8U),
                 static_cast<unsigned long>(err_count));
        MaybeLogSpiStats();
        return;
    }

#if ENABLE_SPI_STRING_TEST
    uint32_t text_seq = 0;
    char text[kTextMaxLen + 1] = {};
    if (ParseTextFrame(rx_buffer, kSpiTransferLen, kFrameTypePynqToEspText, &text_seq, text, sizeof(text))) {
        ++ok_count;

        bool is_new_text = false;
        portENTER_CRITICAL(&g_text_lock);
        if (text_seq != 0U && text_seq != g_rx_text_seq) {
            g_rx_text_seq = text_seq;
            is_new_text = true;
        }
        portEXIT_CRITICAL(&g_text_lock);

        if (is_new_text) {
            ESP_LOGI(TAG, "RX text from PYNQ: seq=%lu text=%s",
                     static_cast<unsigned long>(text_seq),
                     text);
        }
        MaybeLogSpiStats();
        return;
    }
#endif

    adv_status_t adv_status = {};
    if (ParseAdvStatusFrame(rx_buffer, kFrameLen, &adv_status)) {
        ++ok_count;
        if (g_adv_status_queue != nullptr) {
            xQueueSend(g_adv_status_queue, &adv_status, 0);
        }
        MaybeLogSpiStats();
        return;
    }

    adc_waveform_chunk_t adv_chunk = {};
    if (ParseAdvWaveChunkFrame(rx_buffer, kFrameLen, &adv_chunk)) {
        ++ok_count;
        if (kMaxAdvWavesPerPump != 0U && g_adv_wave_queue != nullptr) {
            xQueueSend(g_adv_wave_queue, &adv_chunk, 0);
        }
        MaybeLogSpiStats();
        return;
    }

    adv_harmonic_t adv_harmonic = {};
    if (ParseAdvHarmonicFrame(rx_buffer, kFrameLen, &adv_harmonic)) {
        ++ok_count;
        if (g_adv_harmonic_queue != nullptr) {
            if (xQueueSend(g_adv_harmonic_queue, &adv_harmonic, 0) != pdTRUE) {
                ++dropped_harmonic_count;
            }
        } else {
            ++dropped_harmonic_count;
        }
        MaybeLogSpiStats();
        return;
    }

    freqresp_ui_status_t status = {};
    const bool is_status_frame = ParseFreqRespStatusFrame(rx_buffer, kFrameLen, &status);
    const bool is_point_frame =
        !is_status_frame && ParseFreqRespPointFrame(rx_buffer, kFrameLen, &status);

    if (is_status_frame || is_point_frame) {
        ++ok_count;

        status.packets = ok_count;
        status.frame_errors = err_count;
        status.timeouts = timeout_count;

        if (is_point_frame) {
            portENTER_CRITICAL(&g_measurement_lock);
            g_last_measurement_status = status;
            g_have_measurement_status = true;
            portEXIT_CRITICAL(&g_measurement_lock);

            last_point_index = status.point_index;
            last_freq_hz = status.current_freq_hz;
            point_queue_item_t point_item = {};
            FillPointQueueItem(&status, &point_item);
            if (xQueueSend(g_point_queue, &point_item, 0) != pdTRUE) {
                ++dropped_point_count;
            }
            MaybeUpdatePointQueueHighWater();
        } else {
            status.has_measurement = false;
            xQueueOverwrite(g_status_queue, &status);
        }
        MaybeLogSpiStats();
        return;
    }

    ++err_count;

    MaybeLogInvalidFrame(rx_bits);
    MaybeLogSpiStats();
}
