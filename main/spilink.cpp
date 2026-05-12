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
constexpr uint8_t kFrameTypeAdcWaveform = 0x12;
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

int16_t GetI16(const uint8_t *buffer, size_t offset)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(buffer[offset + 0]) |
        (static_cast<uint16_t>(buffer[offset + 1]) << 8)
    );
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

bool ParseAdcWaveformFrame(const uint8_t *frame, size_t len, adc_waveform_chunk_t *out)
{
    if (frame == nullptr || out == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeAdcWaveform || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];
    if (rx_checksum != expected_checksum) {
        return false;
    }

    size_t o = kFrameHeaderLen;
    out->seq = GetU32(frame, o);                o += 4;
    out->chunk_index = GetU32(frame, o);        o += 4;
    out->chunk_count = GetU32(frame, o);        o += 4;
    out->sample_rate_hz = GetU32(frame, o);     o += 4;
    out->dds_freq_hz = GetU32(frame, o);        o += 4;
    out->total_sample_count = GetU32(frame, o); o += 4;
    out->start_sample_index = GetU32(frame, o); o += 4;
    out->min_mv = GetI32(frame, o);             o += 4;
    out->max_mv = GetI32(frame, o);             o += 4;
    out->mean_mv = GetI32(frame, o);            o += 4;
    out->vpp_mv = GetI32(frame, o);             o += 4;
    out->flags = GetU32(frame, o);              o += 4;

    for (size_t i = 0; i < 30U; ++i) {
        out->samples[i] = GetI16(frame, o);
        o += 2;
    }

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

void UpdateAdcWaveformUiWithLock(const adc_waveform_chunk_t &chunk)
{
    if (esp_lv_adapter_lock(pdMS_TO_TICKS(50)) != ESP_OK) {
        return;
    }

    test_screen_update_adc_waveform_chunk(&chunk);

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
    if (ParseFreqRespStatusFrame(rx_buffer, kFrameLen, &status)) {
        ++ok_count;

        status.packets = ok_count;
        status.frame_errors = err_count;
        status.timeouts = timeout_count;

        ESP_LOGI(TAG,
                 "RX freq response: point=%lu/%lu freq=%luHz vin=%ldmV vout=%ldmV gain=%ld",
                 static_cast<unsigned long>(status.point_index),
                 static_cast<unsigned long>(status.total_points),
                 static_cast<unsigned long>(status.current_freq_hz),
                 static_cast<long>(status.vin_mv),
                 static_cast<long>(status.vout_mv),
                 static_cast<long>(status.gain_x1000));

        UpdateUiWithLock(status);
        return;
    }

    adc_waveform_chunk_t adc_chunk = {};
    if (ParseAdcWaveformFrame(rx_buffer, kFrameLen, &adc_chunk)) {
        ++ok_count;

        ESP_LOGI(TAG,
                 "RX ADC waveform: seq=%lu chunk=%lu/%lu samples=%lu min=%ld max=%ld vpp=%ld flags=0x%08lx",
                 static_cast<unsigned long>(adc_chunk.seq),
                 static_cast<unsigned long>(adc_chunk.chunk_index + 1U),
                 static_cast<unsigned long>(adc_chunk.chunk_count),
                 static_cast<unsigned long>(adc_chunk.total_sample_count),
                 static_cast<long>(adc_chunk.min_mv),
                 static_cast<long>(adc_chunk.max_mv),
                 static_cast<long>(adc_chunk.vpp_mv),
                 static_cast<unsigned long>(adc_chunk.flags));

        UpdateAdcWaveformUiWithLock(adc_chunk);
        return;
    }

    ++err_count;

    ESP_LOGW(TAG,
             "Invalid freq response frame: bits=%u data=%02X %02X %02X %02X %02X %02X %02X %02X err=%lu",
             static_cast<unsigned>(rx_bits),
             rx_buffer[0],
             rx_buffer[1],
             rx_buffer[2],
             rx_buffer[3],
             rx_buffer[4],
             rx_buffer[5],
             rx_buffer[6],
             rx_buffer[7],
             static_cast<unsigned long>(err_count));
#if ENABLE_SPI_STRING_TEST
    UpdateSpiTextUiWithLock(nullptr, 2U);
#endif
}
