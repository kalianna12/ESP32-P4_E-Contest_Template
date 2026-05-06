#include "spilink.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

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
constexpr uint8_t kFrameTypeVoltageMv = 0x02;
constexpr uint8_t kPayloadLen = 0x04;

constexpr size_t kFrameLen = 9;

// 保留 64 字节 transaction buffer。
// 不要改成 9 或 16。STM32 只发前 9 字节，P4 只解析前 9 字节。
constexpr size_t kDmaAlign = 64;
constexpr size_t kRxBufferLen = 64;
constexpr size_t kTxBufferLen = 64;

constexpr uint32_t kDisplayUpdateIntervalMs = 100;

bool spi_ready = false;
uint8_t *rx_buffer = nullptr;
uint8_t *tx_buffer = nullptr;

uint32_t last_value_mv = 0;
uint32_t ok_count = 0;
uint32_t err_count = 0;
uint32_t timeout_count = 0;

TickType_t last_display_tick = 0;

lv_obj_t *line1_label = nullptr;
lv_obj_t *line2_label = nullptr;
lv_obj_t *line3_label = nullptr;

uint8_t Checksum(const uint8_t *frame)
{
    uint8_t checksum = 0;

    for (size_t i = 0; i < 8; ++i) {
        checksum ^= frame[i];
    }

    return checksum;
}

bool ParseVoltageMvFrame(const uint8_t *frame, size_t len, uint32_t *value_mv)
{
    if (frame == nullptr || value_mv == nullptr || len < kFrameLen) {
        return false;
    }

    if (frame[0] != kFrameMagic0 || frame[1] != kFrameMagic1) {
        return false;
    }

    if (frame[2] != kFrameTypeVoltageMv || frame[3] != kPayloadLen) {
        return false;
    }

    if (frame[8] != Checksum(frame)) {
        return false;
    }

    *value_mv = static_cast<uint32_t>(frame[4]) |
                (static_cast<uint32_t>(frame[5]) << 8) |
                (static_cast<uint32_t>(frame[6]) << 16) |
                (static_cast<uint32_t>(frame[7]) << 24);

    return true;
}

bool ShouldUpdateDisplay()
{
    const TickType_t now = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(kDisplayUpdateIntervalMs);

    if ((now - last_display_tick) >= interval) {
        last_display_tick = now;
        return true;
    }

    return false;
}

void FormatMvAsVolt(char *buffer, size_t buffer_len, uint32_t value_mv)
{
    snprintf(buffer,
             buffer_len,
             "%lu.%03lu V",
             static_cast<unsigned long>(value_mv / 1000U),
             static_cast<unsigned long>(value_mv % 1000U));
}

void CreateDisplayIfNeeded()
{
    if (line1_label != nullptr) {
        return;
    }

#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_t *panel = lv_obj_create(screen);
    lv_obj_set_size(panel, 360, 150);
    lv_obj_set_pos(panel, 640, 86);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(panel, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x8FD6C8), LV_PART_MAIN);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x111A24), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN);

    line1_label = lv_label_create(panel);
    line2_label = lv_label_create(panel);
    line3_label = lv_label_create(panel);

    lv_obj_t *labels[] = {line1_label, line2_label, line3_label};

    for (size_t i = 0; i < 3; ++i) {
        lv_obj_set_style_text_color(labels[i], lv_color_hex(0xF5F7FA), LV_PART_MAIN);
        lv_obj_set_style_text_font(labels[i], &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_pos(labels[i], 16, 16 + static_cast<int32_t>(i) * 40);
    }
}

void SpiLink_DisplayText(const char *line1, const char *line2, const char *line3)
{
    if (esp_lv_adapter_lock(pdMS_TO_TICKS(50)) != ESP_OK) {
        return;
    }

    CreateDisplayIfNeeded();

    if (line1_label != nullptr && line2_label != nullptr && line3_label != nullptr) {
        lv_label_set_text(line1_label, line1);
        lv_label_set_text(line2_label, line2);
        lv_label_set_text(line3_label, line3);
    }

    esp_lv_adapter_unlock();
}

void UpdateDisplay()
{
    char voltage_text[32];
    FormatMvAsVolt(voltage_text, sizeof(voltage_text), last_value_mv);

    char line1[96];
    char line2[96];
    char line3[96];

    snprintf(line1, sizeof(line1), "RX: %s", voltage_text);
    snprintf(line2, sizeof(line2), "mV: %lu",
             static_cast<unsigned long>(last_value_mv));
    snprintf(line3, sizeof(line3), "Packets: %lu",
             static_cast<unsigned long>(ok_count));

    SpiLink_DisplayText(line1, line2, line3);
}

void PrepareTxBuffer()
{
    std::memset(tx_buffer, 0x00, kTxBufferLen);

    // 当前 STM32 端可以先不解析 MISO。
    tx_buffer[0] = 0xAC;
    tx_buffer[1] = 0x4B;  // 'K'
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

void HandleValidValue(uint32_t value_mv, size_t rx_bits)
{
    last_value_mv = value_mv;
    ++ok_count;

    ESP_LOGI(TAG,
             "RX=%lu mV packets=%lu err=%lu timeout=%lu bits=%u",
             static_cast<unsigned long>(last_value_mv),
             static_cast<unsigned long>(ok_count),
             static_cast<unsigned long>(err_count),
             static_cast<unsigned long>(timeout_count),
             static_cast<unsigned>(rx_bits));

    if (ShouldUpdateDisplay()) {
        UpdateDisplay();
    }
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

    last_value_mv = 0;
    ok_count = 0;
    err_count = 0;
    timeout_count = 0;
    last_display_tick = 0;
    spi_ready = true;

    UpdateDisplay();

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
    trans.length = kRxBufferLen * 8;  // 保留 64 字节 transaction
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

    uint32_t value_mv = 0;
    const size_t rx_bits = static_cast<size_t>(trans.trans_len);

    // 不依赖 trans_len 判断有效性。
    // 你之前日志里出现过 bits=0 但 rx_buffer 前 9 字节是完整正确帧。
    if (ParseVoltageMvFrame(rx_buffer, kFrameLen, &value_mv)) {
        HandleValidValue(value_mv, rx_bits);
        return;
    }

    ++err_count;

    ESP_LOGW(TAG,
             "Invalid SPI frame: bits=%u data=%02X %02X %02X %02X %02X %02X %02X %02X %02X err=%lu",
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
             static_cast<unsigned long>(err_count));
}