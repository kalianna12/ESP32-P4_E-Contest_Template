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

namespace {

constexpr char TAG[] = "SSVEPSpiLink";

constexpr spi_host_device_t SPI_RX_HOST = SPI3_HOST;
constexpr gpio_num_t SPI_RX_MOSI = GPIO_NUM_21;
constexpr gpio_num_t SPI_RX_MISO = GPIO_NUM_22;
constexpr gpio_num_t SPI_RX_SCLK = GPIO_NUM_23;
constexpr gpio_num_t SPI_RX_CS = GPIO_NUM_3;
constexpr TickType_t SPI_RX_TIMEOUT_TICKS = pdMS_TO_TICKS(200);

constexpr uint8_t kFrameMagic0 = 0xA5;
constexpr uint8_t kFrameMagic1 = 0x5A;
constexpr uint8_t kFrameTypeAdcStatus = 0x10;
constexpr uint8_t kPayloadLen = 112;

constexpr size_t kFrameHeaderLen = 4;
constexpr size_t kFrameLen = kFrameHeaderLen + kPayloadLen + 1;

constexpr size_t kDmaAlign = 64;
constexpr size_t kRxBufferLen = 128;
constexpr size_t kTxBufferLen = 128;

bool spi_ready = false;
uint8_t *rx_buffer = nullptr;
uint8_t *tx_buffer = nullptr;

uint32_t ok_count = 0;
uint32_t err_count = 0;
uint32_t timeout_count = 0;

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

bool ParseAdcStatusFrame(const uint8_t *frame, size_t len, adc_ui_status_t *out)
{
    if (frame == nullptr || out == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeAdcStatus || frame[3] != kPayloadLen) {
        return false;
    }

    const uint8_t expected_checksum = Checksum(frame, kFrameHeaderLen + kPayloadLen);
    const uint8_t rx_checksum = frame[kFrameHeaderLen + kPayloadLen];

    if (rx_checksum != expected_checksum) {
        return false;
    }

    size_t o = 4;

    out->state = static_cast<uint8_t>(GetU32(frame, o));       o += 4;
    out->mode = static_cast<uint8_t>(GetU32(frame, o));        o += 4;
    out->source = static_cast<uint8_t>(GetU32(frame, o));      o += 4;
    out->monitor_ok = static_cast<uint8_t>(GetU32(frame, o));  o += 4;
    out->progress_permille = GetU32(frame, o);                 o += 4;
    out->elapsed_ms = GetU32(frame, o);                        o += 4;

    out->sample_index = GetU32(frame, o);       o += 4;
    out->total_samples = GetU32(frame, o);      o += 4;

    out->dut_adc_code = GetU32(frame, o);       o += 4;
    out->dut_adc_bits = GetU32(frame, o);       o += 4;
    out->dut_adc_avg_x1000 = GetU32(frame, o);  o += 4;
    out->dut_conversion_time_ns = GetU32(frame, o); o += 4;

    out->input_mv = GetU32(frame, o);           o += 4;
    out->stm32_adc_raw12 = GetU32(frame, o);    o += 4;
    out->stm32_adc_mv = GetU32(frame, o);       o += 4;

    out->offset_error_lsb_x1000 = GetI32(frame, o); o += 4;
    out->gain_error_lsb_x1000 = GetI32(frame, o);   o += 4;
    out->gain_error_ppm = GetI32(frame, o);         o += 4;
    out->dnl_min_x1000 = GetI32(frame, o);          o += 4;
    out->dnl_max_x1000 = GetI32(frame, o);          o += 4;
    out->inl_min_x1000 = GetI32(frame, o);          o += 4;
    out->inl_max_x1000 = GetI32(frame, o);          o += 4;
    out->missing_codes = GetU32(frame, o);          o += 4;

    out->snr_db_x100 = GetI32(frame, o);       o += 4;
    out->sinad_db_x100 = GetI32(frame, o);     o += 4;
    out->enob_x100 = GetI32(frame, o);         o += 4;
    out->sfdr_db_x100 = GetI32(frame, o);      o += 4;
    out->thd_db_x100 = GetI32(frame, o);       o += 4;

    return true;
}

void PrepareTxBuffer()
{
    std::memset(tx_buffer, 0x00, kTxBufferLen);
    tx_buffer[0] = 0xAC;
    tx_buffer[1] = 0x4B;
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

void UpdateUiWithLock(const adc_ui_status_t &status)
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
    spi_ready = true;

    ESP_LOGI(TAG,
             "SPI slave ready: host=SPI3 MOSI=%d MISO=%d SCLK=%d CS=%d rx_len=%u",
             static_cast<int>(SPI_RX_MOSI),
             static_cast<int>(SPI_RX_MISO),
             static_cast<int>(SPI_RX_SCLK),
             static_cast<int>(SPI_RX_CS),
             static_cast<unsigned>(kRxBufferLen));
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

    adc_ui_status_t status = {};
    const size_t rx_bits = static_cast<size_t>(trans.trans_len);

    if (ParseAdcStatusFrame(rx_buffer, kFrameLen, &status)) {
        ++ok_count;

        status.packets = ok_count;
        status.frame_errors = err_count;
        status.timeouts = timeout_count;

        ESP_LOGI(TAG,
                 "RX ADC status: sample=%lu/%lu vin=%lu mV dut=%lu bits=%u",
                 static_cast<unsigned long>(status.sample_index),
                 static_cast<unsigned long>(status.total_samples),
                 static_cast<unsigned long>(status.input_mv),
                 static_cast<unsigned long>(status.dut_adc_code),
                 static_cast<unsigned>(rx_bits));

        UpdateUiWithLock(status);
        return;
    }

    ++err_count;

    ESP_LOGW(TAG,
             "Invalid ADC status frame: bits=%u data=%02X %02X %02X %02X %02X %02X %02X %02X err=%lu",
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
}
