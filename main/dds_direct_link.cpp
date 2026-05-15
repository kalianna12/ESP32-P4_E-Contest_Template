#include "dds_direct_link.h"

#include <algorithm>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "DdsDirect";

constexpr spi_host_device_t DDS_SPI_HOST = SPI2_HOST;
constexpr gpio_num_t DDS_SPI_SCLK = GPIO_NUM_33;
constexpr gpio_num_t DDS_SPI_MOSI = GPIO_NUM_54;
constexpr gpio_num_t DDS_SPI_MISO = GPIO_NUM_48;
constexpr gpio_num_t DDS_SPI_CS = GPIO_NUM_32;
constexpr int DDS_SPI_CLOCK_HZ = 1 * 1000 * 1000;

constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0x5A;
constexpr uint8_t kPayloadLen = 112;
constexpr size_t kFrameLen = 128;
constexpr size_t kChecksumOffset = 116;
constexpr uint32_t kDefaultSampleCount = 1024;
constexpr uint32_t kDefaultSampleRateHz = 100000;
constexpr uint32_t kSamplesPerChunk = 30;

spi_device_handle_t g_dds_spi = nullptr;
SemaphoreHandle_t g_lock = nullptr;
uint8_t *g_tx = nullptr;
uint8_t *g_rx = nullptr;
uint32_t g_seq = 0xE5000000U;
bool g_ready = false;

void PutU32(uint8_t *frame, size_t offset, uint32_t value)
{
    frame[offset + 0] = static_cast<uint8_t>(value & 0xFFU);
    frame[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    frame[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
    frame[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
}

uint32_t GetU32(const uint8_t *frame, size_t offset)
{
    return static_cast<uint32_t>(frame[offset + 0]) |
           (static_cast<uint32_t>(frame[offset + 1]) << 8) |
           (static_cast<uint32_t>(frame[offset + 2]) << 16) |
           (static_cast<uint32_t>(frame[offset + 3]) << 24);
}

void PutI16(uint8_t *frame, size_t offset, int16_t value)
{
    const uint16_t u = static_cast<uint16_t>(value);
    frame[offset + 0] = static_cast<uint8_t>(u & 0xFFU);
    frame[offset + 1] = static_cast<uint8_t>((u >> 8) & 0xFFU);
}

uint8_t Checksum(const uint8_t *frame)
{
    uint8_t chk = 0;
    for (size_t i = 0; i < kChecksumOffset; ++i) {
        chk ^= frame[i];
    }
    return chk;
}

uint32_t NextSeq()
{
    ++g_seq;
    if (g_seq == 0U) {
        ++g_seq;
    }
    return g_seq;
}

void BeginFrame(uint8_t type, uint32_t seq)
{
    std::memset(g_tx, 0x00, kFrameLen);
    std::memset(g_rx, 0x00, kFrameLen);
    g_tx[0] = kMagic0;
    g_tx[1] = kMagic1;
    g_tx[2] = type;
    g_tx[3] = kPayloadLen;
    PutU32(g_tx, 4, seq);
}

void FinishFrame()
{
    g_tx[kChecksumOffset] = Checksum(g_tx);
}

void LogAck(const char *tag)
{
    if (g_rx[0] != kMagic0 || g_rx[1] != kMagic1 || g_rx[2] != 0xD2 ||
        g_rx[3] != kPayloadLen || g_rx[kChecksumOffset] != Checksum(g_rx)) {
        ESP_LOGW(TAG,
                 "%s ack invalid: head=%02X %02X %02X %02X chk=%02X expect=%02X",
                 tag,
                 g_rx[0],
                 g_rx[1],
                 g_rx[2],
                 g_rx[3],
                 g_rx[kChecksumOffset],
                 Checksum(g_rx));
        return;
    }

    ESP_LOGI(TAG,
             "%s ack seq=0x%08lX cmd=0x%08lX freq=%lu flags=0x%08lX",
             tag,
             static_cast<unsigned long>(GetU32(g_rx, 4)),
             static_cast<unsigned long>(GetU32(g_rx, 8)),
             static_cast<unsigned long>(GetU32(g_rx, 12)),
             static_cast<unsigned long>(GetU32(g_rx, 16)));
}

bool TransferFrame(const char *tag)
{
    spi_transaction_t trans = {};
    trans.length = kFrameLen * 8;
    trans.tx_buffer = g_tx;
    trans.rx_buffer = g_rx;

    const esp_err_t err = spi_device_polling_transmit(g_dds_spi, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s transmit failed: %s", tag, esp_err_to_name(err));
        return false;
    }

    LogAck(tag);
    return true;
}

bool EnsureReady()
{
    if (g_ready) {
        return true;
    }
    return DdsDirect_Init();
}

int16_t ClampI16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return static_cast<int16_t>(value);
}

} // namespace

bool DdsDirect_Init(void)
{
    if (g_ready) {
        return true;
    }
    if (g_dds_spi != nullptr) {
        g_ready = true;
        return true;
    }

    if (g_lock == nullptr) {
        g_lock = xSemaphoreCreateMutex();
        if (g_lock == nullptr) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return false;
        }
    }

    if (g_tx == nullptr) {
        g_tx = static_cast<uint8_t *>(heap_caps_aligned_alloc(
            64, kFrameLen, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    }
    if (g_rx == nullptr) {
        g_rx = static_cast<uint8_t *>(heap_caps_aligned_alloc(
            64, kFrameLen, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    }
    if (g_tx == nullptr || g_rx == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers");
        return false;
    }

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = DDS_SPI_MOSI;
    bus_cfg.miso_io_num = DDS_SPI_MISO;
    bus_cfg.sclk_io_num = DDS_SPI_SCLK;
    bus_cfg.quadwp_io_num = GPIO_NUM_NC;
    bus_cfg.quadhd_io_num = GPIO_NUM_NC;
    bus_cfg.max_transfer_sz = kFrameLen;

    esp_err_t err = spi_bus_initialize(DDS_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI2_HOST is already initialized; DDS direct SPI pins may conflict");
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = DDS_SPI_CLOCK_HZ;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = DDS_SPI_CS;
    dev_cfg.queue_size = 1;

    err = spi_bus_add_device(DDS_SPI_HOST, &dev_cfg, &g_dds_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        g_dds_spi = nullptr;
        return false;
    }

    g_ready = true;
    ESP_LOGI(TAG,
             "DDS direct SPI ready: host=SPI2 SCLK=%d MOSI=%d MISO=%d CS=%d mode=0 clk=%dHz len=%u",
             static_cast<int>(DDS_SPI_SCLK),
             static_cast<int>(DDS_SPI_MOSI),
             static_cast<int>(DDS_SPI_MISO),
             static_cast<int>(DDS_SPI_CS),
             DDS_SPI_CLOCK_HZ,
             static_cast<unsigned>(kFrameLen));
    return true;
}

bool DdsDirect_SendWave(const int16_t *samples, uint32_t sample_count, uint32_t sample_rate_hz)
{
    if (samples == nullptr || sample_count == 0U || sample_count > 1024U) {
        ESP_LOGE(TAG, "Invalid wave: samples=%p count=%lu", samples,
                 static_cast<unsigned long>(sample_count));
        return false;
    }
    if (sample_rate_hz == 0U) {
        sample_rate_hz = kDefaultSampleRateHz;
    }
    if (!EnsureReady()) {
        return false;
    }

    xSemaphoreTake(g_lock, portMAX_DELAY);

    const uint32_t seq = NextSeq();
    bool ok = true;

    BeginFrame(0xD3, seq);
    PutU32(g_tx, 8, sample_count);
    PutU32(g_tx, 12, sample_rate_hz);
    FinishFrame();
    ok = TransferFrame("D3");
    vTaskDelay(pdMS_TO_TICKS(1));

    const uint32_t chunk_count = (sample_count + kSamplesPerChunk - 1U) / kSamplesPerChunk;
    for (uint32_t chunk = 0; ok && chunk < chunk_count; ++chunk) {
        const uint32_t start_index = chunk * kSamplesPerChunk;
        const uint32_t remaining = sample_count - start_index;
        const uint32_t count = std::min<uint32_t>(remaining, kSamplesPerChunk);

        BeginFrame(0xD4, seq);
        PutU32(g_tx, 8, chunk);
        PutU32(g_tx, 12, start_index);
        PutU32(g_tx, 16, count);
        for (uint32_t i = 0; i < count; ++i) {
            PutI16(g_tx, 20 + i * 2U, samples[start_index + i]);
        }
        FinishFrame();
        ok = TransferFrame("D4");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (ok) {
        BeginFrame(0xD5, seq);
        FinishFrame();
        ok = TransferFrame("D5");
    }

    xSemaphoreGive(g_lock);

    ESP_LOGI(TAG,
             "DDS direct send %s: seq=0x%08lX samples=%lu rate=%lu chunks=%lu",
             ok ? "done" : "failed",
             static_cast<unsigned long>(seq),
             static_cast<unsigned long>(sample_count),
             static_cast<unsigned long>(sample_rate_hz),
             static_cast<unsigned long>(chunk_count));
    return ok;
}

bool DdsDirect_SendSquareTest(void)
{
    static int16_t samples[kDefaultSampleCount];
    for (uint32_t i = 0; i < kDefaultSampleCount; ++i) {
        samples[i] = (i & 32U) ? static_cast<int16_t>(6000) : static_cast<int16_t>(-6000);
    }
    return DdsDirect_SendWave(samples, kDefaultSampleCount, kDefaultSampleRateHz);
}

bool DdsDirect_SendTriangleTest(void)
{
    static int16_t samples[kDefaultSampleCount];
    constexpr int32_t amp = 6000;
    for (uint32_t i = 0; i < kDefaultSampleCount; ++i) {
        const uint32_t phase = i & 63U;
        int32_t value = 0;
        if (phase < 32U) {
            value = -amp + static_cast<int32_t>((phase * (2U * amp)) / 31U);
        } else {
            value = amp - static_cast<int32_t>(((phase - 32U) * (2U * amp)) / 31U);
        }
        samples[i] = ClampI16(value);
    }
    return DdsDirect_SendWave(samples, kDefaultSampleCount, kDefaultSampleRateHz);
}

bool ReconFromCapture_NoFftPassthrough(void)
{
    ESP_LOGW(TAG, "ReconFromCapture_NoFftPassthrough reserved; not implemented yet");
    return false;
}

bool ReconFromCapture_FftCompensate(void)
{
    ESP_LOGW(TAG, "ReconFromCapture_FftCompensate reserved; not implemented yet");
    return false;
}
