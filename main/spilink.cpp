#include "spilink.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
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

bool spi_ready = false;
uint8_t *rx_buffer = nullptr;
uint8_t *tx_buffer = nullptr;

uint32_t ok_count = 0;
uint32_t err_count = 0;
uint32_t timeout_count = 0;
freqresp_ui_status_t g_last_measurement_status = {};
bool g_have_measurement_status = false;

portMUX_TYPE g_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
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

int32_t RawVppCodeToMv(uint32_t code)
{
    return static_cast<int32_t>((static_cast<uint64_t>(code) * 10000ULL) / 4095ULL);
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
    out->gain_x1000 = (raw_a_vpp_code != 0U) ?
        static_cast<int32_t>((static_cast<uint64_t>(raw_b_vpp_code) * 1000ULL) / raw_a_vpp_code) :
        frame_gain_x1000;
    out->theory_gain_x1000 = 0;
    out->error_x10 = 0;
    out->phase_deg_x10 = GetI32(frame, 44);
    out->cutoff_freq_hz = 0;

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

void UpdateSpiTextUiWithLock(const char *text, uint8_t link_state)
{
    if (esp_lv_adapter_lock(pdMS_TO_TICKS(50)) != ESP_OK) {
        return;
    }

    test_screen_update_spi_text_test(text, link_state);

    esp_lv_adapter_unlock();
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

void UpdateUiWithLock(const freqresp_ui_status_t &status)
{
    if (esp_lv_adapter_lock(pdMS_TO_TICKS(50)) != ESP_OK) {
        return;
    }

    test_screen_update_measurement(&status);

    esp_lv_adapter_unlock();
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
    slave_cfg.queue_size = 1;
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
        FreeBuffers();
        return;
    }

    ok_count = 0;
    err_count = 0;
    timeout_count = 0;
    g_last_measurement_status = {};
    g_have_measurement_status = false;
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
#if ENABLE_SPI_STRING_TEST
        UpdateSpiTextUiWithLock(nullptr, 0U);
#endif
        return;
    }

    if (err != ESP_OK) {
        ++err_count;
        ESP_LOGW(TAG,
                 "spi_slave_transmit failed: %s err=%lu",
                 esp_err_to_name(err),
                 static_cast<unsigned long>(err_count));
        vTaskDelay(pdMS_TO_TICKS(10));
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
            UpdateSpiTextUiWithLock(text, 1U);
        } else {
            UpdateSpiTextUiWithLock(nullptr, 1U);
        }
        return;
    }
#endif

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
            const uint32_t raw_a_vpp_code = GetU32(rx_buffer, 32) & 0x0FFFU;
            const uint32_t raw_b_vpp_code = GetU32(rx_buffer, 36) & 0x0FFFU;
            ESP_LOGI(TAG,
                     "RX point: idx=%lu/%lu freq=%luHz rawA=%lu rawB=%lu vin_mv=%ld vout_mv=%ld gain=%ld phase=%ld",
                     static_cast<unsigned long>(status.point_index),
                     static_cast<unsigned long>(status.total_points),
                     static_cast<unsigned long>(status.current_freq_hz),
                     static_cast<unsigned long>(raw_a_vpp_code),
                     static_cast<unsigned long>(raw_b_vpp_code),
                     static_cast<long>(status.vin_mv),
                     static_cast<long>(status.vout_mv),
                     static_cast<long>(status.gain_x1000),
                     static_cast<long>(status.phase_deg_x10));

            g_last_measurement_status = status;
            g_have_measurement_status = true;
            UpdateUiWithLock(status);
        } else {
            ESP_LOGI(TAG,
                     "RX status: point=%lu/%lu freq=%luHz state=%u progress=%lu",
                     static_cast<unsigned long>(status.point_index),
                     static_cast<unsigned long>(status.total_points),
                     static_cast<unsigned long>(status.current_freq_hz),
                     static_cast<unsigned>(status.state),
                     static_cast<unsigned long>(status.progress_permille));

            freqresp_ui_status_t merged = {};
            if (g_have_measurement_status) {
                merged = g_last_measurement_status;
            }

            merged.state = status.state;
            merged.mode = status.mode;
            merged.filter_type = status.filter_type;
            merged.link_ok = status.link_ok;
            merged.progress_permille = status.progress_permille;
            merged.start_freq_hz = status.start_freq_hz;
            merged.stop_freq_hz = status.stop_freq_hz;
            merged.step_freq_hz = status.step_freq_hz;
            merged.single_freq_hz = status.single_freq_hz;
            if (!g_have_measurement_status) {
                merged.current_freq_hz = status.current_freq_hz;
                merged.point_index = status.point_index;
                merged.total_points = status.total_points;
            }
            merged.cutoff_freq_hz = status.cutoff_freq_hz;
            merged.packets = ok_count;
            merged.frame_errors = err_count;
            merged.timeouts = timeout_count;

            UpdateUiWithLock(merged);
        }
        return;
    }

    ++err_count;

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
#if ENABLE_SPI_STRING_TEST
    UpdateSpiTextUiWithLock(nullptr, 2U);
#endif
}
