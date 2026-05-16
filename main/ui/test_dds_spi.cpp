#include "test_dds_spi.h"

#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "dds_direct_link.h"
#include "spilink.h"

#include <stdio.h>
#include <string.h>

// ─────────────────────────────────────────
//  Test configuration
// ─────────────────────────────────────────
#define DDS_TEST_USE_BITBANG       1      // 1=bitbang, 0=SPI hardware (bitbang has more diagnostics)
#define DDS_TEST_ECHO_FRAMES       16
#define DDS_TEST_SQUARE_AMPLITUDE  6000
#define DDS_TEST_SAMPLE_RATE_HZ    100000
#define DDS_TEST_SINGLE_FREQ_HZ    1000
#define DDS_TEST_SWEEP_START_HZ    1000
#define DDS_TEST_SWEEP_STOP_HZ     2000
#define DDS_TEST_SWEEP_STEP_HZ     500
#define DDS_TEST_ACK_PARSE_SLOTS    8     // show last N ACKs in the UI log

#define LOG_LINES                  28
#define LOG_LINE_BYTES             80

// ─────────────────────────────────────────
//  Color shortcuts
// ─────────────────────────────────────────
#define C_TEXT   0xF8FAFC
#define C_SUB    0xCBD5E1
#define C_GREEN  0x86EFAC
#define C_YELLOW 0xFBBF24
#define C_RED    0xFCA5A5
#define C_BLUE   0x93C5FD
#define C_BG     0x0B1020

// ─────────────────────────────────────────
//  Test result
// ─────────────────────────────────────────
typedef enum {
    RESULT_NONE = 0,
    RESULT_PASS,
    RESULT_FAIL,
    RESULT_RUNNING,
    RESULT_IDLE,
} test_result_t;

typedef struct {
    const char *name;
    test_result_t result;
    uint32_t tick_ms;
} test_slot_t;

static test_slot_t g_slots[8] = {};
static int g_slot_count = 0;
static int g_current_test = -1;

// Log lines (circular buffer)
static char g_log[LOG_LINES][LOG_LINE_BYTES];
static int g_log_head = 0;
static int g_log_count = 0;

// UI widgets (updated in pump)
static lv_obj_t *g_log_area = nullptr;
static lv_obj_t *g_result_labels[8] = {};
static lv_obj_t *g_link_status = nullptr;
static lv_obj_t *g_link_stat_label = nullptr;
static lv_obj_t *g_pass_count_label = nullptr;
static lv_obj_t *g_fail_count_label = nullptr;
static lv_obj_t *g_overall_label = nullptr;
static bool g_page_active = false;
static uint32_t g_pass_count = 0;
static uint32_t g_fail_count = 0;

// ─────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────
static void log_add(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    char buf[LOG_LINE_BYTES];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    int line = g_log_head;
    strncpy(g_log[line], buf, LOG_LINE_BYTES - 1);
    g_log[line][LOG_LINE_BYTES - 1] = '\0';
    g_log_head = (g_log_head + 1) % LOG_LINES;
    if (g_log_count < LOG_LINES) {
        ++g_log_count;
    }

    if (g_log_area != nullptr) {
        // Build full text from oldest to newest
        char full[LOG_LINES * LOG_LINE_BYTES] = {};
        int out_pos = 0;
        int start = (g_log_count < LOG_LINES) ? 0 : g_log_head;
        for (int i = 0; i < g_log_count; ++i) {
            int idx = (start + i) % LOG_LINES;
            int len = strlen(g_log[idx]);
            if (out_pos + len + 1 < (int)sizeof(full)) {
                memcpy(full + out_pos, g_log[idx], len);
                out_pos += len;
                full[out_pos++] = '\n';
            }
        }
        full[out_pos] = '\0';
        lv_textarea_set_text(g_log_area, full[0] ? full : "");
        lv_textarea_set_cursor_pos(g_log_area, LV_TEXTAREA_CURSOR_POS_NONE);
    }
}

static void log_separator(const char *title)
{
    log_add("═══════════════════════════════════");
    log_add("  %s", title);
    log_add("═══════════════════════════════════");
}

// Mark a slot as PASS/FAIL and bump counters
static void slot_set_result(int slot, test_result_t r)
{
    if (slot < 0 || slot >= g_slot_count) return;
    g_slots[slot].result = r;
    if (r == RESULT_PASS) {
        ++g_pass_count;
    } else if (r == RESULT_FAIL) {
        ++g_fail_count;
    }
}

// ─────────────────────────────────────────
//  Test 0: DDS Direct SPI Echo
//  Target: ESP32-P4 → PYNQDDS (ESP SPI port)
//  What it tests: DDS SPI link, frame parsing, ACK recovery
//  Success: All echo frames acknowledged correctly
// ─────────────────────────────────────────
static void test_dds_echo(void)
{
    log_separator("Test 0: DDS Direct SPI Echo");
    log_add("Sending %u echo frames to PYNQDDS...", DDS_TEST_ECHO_FRAMES);

    if (!DdsDirect_Init()) {
        log_add("FAIL: DdsDirect_Init failed");
        return;
    }

    uint32_t pass = 0, fail = 0;
    for (uint32_t n = 0; n < DDS_TEST_ECHO_FRAMES; ++n) {
        uint32_t seq = 0xE5000000U + n;
        // Build echo frame
        memset(g_tx, 0, 128);
        g_tx[0] = 0xA5; g_tx[1] = 0x5A; g_tx[2] = 0xE0; g_tx[3] = 112;
        g_tx[4] = (seq >> 0) & 0xFF; g_tx[5] = (seq >> 8) & 0xFF;
        g_tx[6] = (seq >> 16) & 0xFF; g_tx[7] = (seq >> 24) & 0xFF;
        g_tx[8] = 0x11; g_tx[9] = 0x22; g_tx[10] = 0x33; g_tx[11] = 0x44;
        g_tx[12] = 0x55; g_tx[13] = 0x66; g_tx[14] = 0x77; g_tx[15] = 0x88;
        for (int i = 20; i < 116; ++i) g_tx[i] = (uint8_t)(0xA0 + i);
        // Checksum
        uint8_t chk = 0;
        for (int i = 0; i < 116; ++i) chk ^= g_tx[i];
        g_tx[116] = chk;

        TransferFrame("ECHO");
        vTaskDelay(pdMS_TO_TICKS(20));

        // Check if g_rx looks like a valid ACK
        bool looks_ack = (g_rx[0] == 0xA5 && g_rx[1] == 0x5A && g_rx[2] == 0xD2 && g_rx[3] == 112);
        if (looks_ack) ++pass; else ++fail;

        if (n < 4 || n == DDS_TEST_ECHO_FRAMES - 1) {
            log_add("  frame %02u: %s  rx_head=%02X%02X%02X%02X chk=%02X",
                    n, looks_ack ? "ACK" : "???",
                    g_rx[0], g_rx[1], g_rx[2], g_rx[3], g_rx[116]);
        }
    }

    log_add("  Echo result: %lu OK, %lu bad", pass, fail);
    if (fail == 0) {
        log_add("RESULT: PASS - DDS SPI link is clean");
    } else {
        log_add("RESULT: FAIL - %lu/%lu frames not acknowledged", fail, DDS_TEST_ECHO_FRAMES);
    }
}

// ─────────────────────────────────────────
//  Test 1: DDS Direct Square Wave
//  Target: ESP32-P4 → PYNQDDS (ESP SPI port)
//  What it tests: D3+D4+D5 sequence, wave RAM write, DAC output
//  Success: Square wave visible on oscilloscope
// ─────────────────────────────────────────
static void test_dds_square(void)
{
    log_separator("Test 1: DDS Direct Square Wave");
    log_add("Building 4096-sample square wave (amp=%d)...", DDS_TEST_SQUARE_AMPLITUDE);

    static int16_t samples[4096];
    for (uint32_t i = 0; i < 4096; ++i) {
        samples[i] = (i & 32) ? DDS_TEST_SQUARE_AMPLITUDE : -DDS_TEST_SQUARE_AMPLITUDE;
    }

    log_add("Sending D3+D4 chunks to PYNQDDS...");
    bool ok = DdsDirect_SendWave(samples, 4096, DDS_TEST_SAMPLE_RATE_HZ);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (ok) {
        log_add("RESULT: PASS - Wave sent to PYNQDDS");
        log_add("  -> Check oscilloscope on DAC output");
        log_add("  -> Square wave should be visible");
        log_add("  -> LED2 (arb_playing) should be ON on PYNQDDS board");
    } else {
        log_add("RESULT: FAIL - DdsDirect_SendWave returned false");
    }
}

// ─────────────────────────────────────────
//  Test 2: DDS Direct Triangle Wave
//  Target: ESP32-P4 → PYNQDDS (ESP SPI port)
//  What it tests: Different waveform shape, full D3→D4→D5 flow
// ─────────────────────────────────────────
static void test_dds_triangle(void)
{
    log_separator("Test 2: DDS Direct Triangle Wave");

    static int16_t samples[4096];
    for (uint32_t i = 0; i < 4096; ++i) {
        uint32_t phase = i & 63;
        int32_t val = 0;
        if (phase < 32) {
            val = -DDS_TEST_SQUARE_AMPLITUDE + (int32_t)((phase * (2 * DDS_TEST_SQUARE_AMPLITUDE)) / 31);
        } else {
            val = DDS_TEST_SQUARE_AMPLITUDE - (int32_t)(((phase - 32) * (2 * DDS_TEST_SQUARE_AMPLITUDE)) / 31);
        }
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        samples[i] = (int16_t)val;
    }

    bool ok = DdsDirect_SendWave(samples, 4096, DDS_TEST_SAMPLE_RATE_HZ);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (ok) {
        log_add("RESULT: PASS - Triangle wave sent");
    } else {
        log_add("RESULT: FAIL - DdsDirect_SendWave returned false");
    }
}

// ─────────────────────────────────────────
//  Test 3: SPI-A Link Poll (NOP frames)
//  Target: ESP32-P4 → PYNQADC (SPI-A slave port)
//  What it tests: ESP→ADC SPI link, no command sent, just poll
//  Success: Valid status frame received back from PYNQADC
// ─────────────────────────────────────────
static void test_spi_a_poll(void)
{
    log_separator("Test 3: SPI-A Link Poll (NOP)");
    log_add("Sending NOP frames to PYNQADC via SPI-A...");

    // Use the existing command queue to send NOP
    // First, let's just pump several SPI transactions and check what comes back
    uint32_t pass = 0, err = 0;
    for (int n = 0; n < 20; ++n) {
        // The SpiLink_Task will send whatever is queued.
        // Send a CLEAR_TABLE NOP (cmd=8) which should not affect measurement state.
        SpiLink_SetPendingCommand(CMD_CLEAR_TABLE, 0U, 0U);
        vTaskDelay(pdMS_TO_TICKS(80));

        // Check stats
        spilink_stats_t stats;
        SpiLink_GetStats(&stats);
        if (stats.frame_errors == 0 && stats.bad_first_bytes == 0) {
            ++pass;
        } else {
            ++err;
        }

        if (n < 4) {
            log_add("  poll %02d: OK=%lu ERR=%lu BFB=%lu",
                    n + 1, stats.frame_errors, stats.bad_first_bytes, stats.rx_ok);
        }
    }

    log_add("  Poll result: %lu clean, %lu errors", pass, err);
    if (err == 0) {
        log_add("RESULT: PASS - SPI-A link is clean");
    } else {
        log_add("RESULT: FAIL - %lu/%lu polls had frame errors", err, pass + err);
    }
}

// ─────────────────────────────────────────
//  Test 4: Basic Sweep (via SPI-A → PYNQADC → PYNQDDS)
//  Target: Full chain ESP→ADC→DDS
//  What it tests: Sweep config, START, DDS ACK verification, state machine
//  Success: PYNQADC enters SCANNING state and DDS outputs sine wave
// ─────────────────────────────────────────
static void test_basic_sweep(void)
{
    log_separator("Test 4: Basic Sweep (Full Chain)");
    log_add("Queueing: SET_MODE(SWEEP), SET_START(1kHz), SET_STOP(2kHz), SET_STEP(500Hz), START");

    // Clear
    SpiLink_SetPendingCommand(CMD_CLEAR_TABLE, 0U, 0U);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Config + Start
    SpiLink_SetPendingCommand(CMD_SET_MODE, MODE_SWEEP, 0U);
    SpiLink_SetPendingCommand(CMD_SET_START_FREQ, DDS_TEST_SWEEP_START_HZ, 0U);
    SpiLink_SetPendingCommand(CMD_SET_STOP_FREQ, DDS_TEST_SWEEP_STOP_HZ, 0U);
    SpiLink_SetPendingCommand(CMD_SET_STEP_FREQ, DDS_TEST_SWEEP_STEP_HZ, 0U);
    SpiLink_SetPendingCommand(CMD_START, 0U, 0U);

    log_add("Waiting 5 seconds for sweep to run...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    spilink_stats_t stats;
    SpiLink_GetStats(&stats);
    log_add("  After 5s: rx_ok=%lu frame_err=%lu timeout=%lu bad_first=%lu",
            stats.rx_ok, stats.frame_errors, stats.timeouts, stats.bad_first_bytes);

    // Check if we got any point data
    uint32_t q = SpiLink_PointQueueWaiting();
    log_add("  Points in queue: %lu", q);

    if (stats.frame_errors == 0 && stats.bad_first_bytes == 0 && q > 0) {
        log_add("RESULT: PASS - Sweep is running, points arriving");
    } else if (stats.frame_errors == 0 && stats.bad_first_bytes == 0) {
        log_add("RESULT: MARGINAL - SPI clean but no points yet (freq may be out of range)");
    } else {
        log_add("RESULT: FAIL - Frame errors or BFB detected during sweep");
    }

    // Stop
    log_add("Sending STOP...");
    SpiLink_SetPendingCommand(CMD_STOP, 0U, 0U);
    vTaskDelay(pdMS_TO_TICKS(200));
}

// ─────────────────────────────────────────
//  Test 5: Single-Frequency (via SPI-A)
//  Target: ESP→ADC→DDS single tone
//  What it tests: CMD_SET_SINGLE_FREQ + CMD_START
// ─────────────────────────────────────────
static void test_basic_single(void)
{
    log_separator("Test 5: Single-Frequency (Full Chain)");
    log_add("Queueing: SET_MODE(SINGLE), SET_SINGLE(1kHz), START");

    SpiLink_SetPendingCommand(CMD_CLEAR_TABLE, 0U, 0U);
    vTaskDelay(pdMS_TO_TICKS(100));

    SpiLink_SetPendingCommand(CMD_SET_MODE, MODE_SINGLE, 0U);
    SpiLink_SetPendingCommand(CMD_SET_SINGLE_FREQ, DDS_TEST_SINGLE_FREQ_HZ, 0U);
    SpiLink_SetPendingCommand(CMD_START, 0U, 0U);

    log_add("Waiting 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    spilink_stats_t stats;
    SpiLink_GetStats(&stats);
    uint32_t q = SpiLink_PointQueueWaiting();

    log_add("  rx_ok=%lu frame_err=%lu timeout=%lu BFB=%lu points=%lu",
            stats.rx_ok, stats.frame_errors, stats.timeouts, stats.bad_first_bytes, q);

    if (stats.frame_errors == 0 && stats.bad_first_bytes == 0 && q > 0) {
        log_add("RESULT: PASS - Single freq working, point received");
    } else if (stats.frame_errors == 0 && stats.bad_first_bytes == 0) {
        log_add("RESULT: MARGINAL - SPI clean but no points");
    } else {
        log_add("RESULT: FAIL - Errors during single-freq test");
    }

    SpiLink_SetPendingCommand(CMD_STOP, 0U, 0U);
    vTaskDelay(pdMS_TO_TICKS(200));
}

// ─────────────────────────────────────────
//  Test 6: DDS Arb → Basic handover
//  Target: Force switch from arbitrary-wave back to basic DDS
//  What it tests: Does basic sweep command properly stop arb_playing?
// ─────────────────────────────────────────
static void test_arb_to_basic(void)
{
    log_separator("Test 6: Arb→Basic Handover");
    log_add("Step 1: Send square wave via DDS Direct (arbitrary path)");
    log_add("  Expected: LED0=ON, LED2=ON on PYNQDDS board");

    static int16_t samples[4096];
    for (uint32_t i = 0; i < 4096; ++i) {
        samples[i] = (i & 32) ? 6000 : -6000;
    }
    DdsDirect_SendWave(samples, 4096, DDS_TEST_SAMPLE_RATE_HZ);
    vTaskDelay(pdMS_TO_TICKS(500));

    log_add("Step 2: Trigger basic single-freq via SPI-A");
    log_add("  Expected: DDS ACK should clear arb_playing, DAC switches to sine");
    log_add("  Check: LED0 and LED2 should go OFF on PYNQDDS board");

    SpiLink_SetPendingCommand(CMD_CLEAR_TABLE, 0U, 0U);
    vTaskDelay(pdMS_TO_TICKS(100));
    SpiLink_SetPendingCommand(CMD_SET_MODE, MODE_SINGLE, 0U);
    SpiLink_SetPendingCommand(CMD_SET_SINGLE_FREQ, 2000U, 0U);
    SpiLink_SetPendingCommand(CMD_START, 0U, 0U);
    vTaskDelay(pdMS_TO_TICKS(3000));

    spilink_stats_t stats;
    SpiLink_GetStats(&stats);
    log_add("  frame_err=%lu BFB=%lu", stats.frame_errors, stats.bad_first_bytes);
    log_add("RESULT: PASS - Observation only, check LED state on PYNQDDS");
    log_add("  LED0 should be OFF (not arb_mode)");
    log_add("  LED2 should be OFF (not arb_playing)");
    log_add("  Oscilloscope should show sine wave (basic DDS path)");

    SpiLink_SetPendingCommand(CMD_STOP, 0U, 0U);
    vTaskDelay(pdMS_TO_TICKS(200));
}

// ─────────────────────────────────────────
//  Test sequence definition
// ─────────────────────────────────────────
typedef void (*test_fn)(void);

static const test_fn g_test_sequence[] = {
    test_dds_echo,
    test_dds_square,
    test_dds_triangle,
    test_spi_a_poll,
    test_basic_sweep,
    test_basic_single,
    test_arb_to_basic,
};

static const char *g_test_names[] = {
    "0:DDS Echo",
    "1:DDS Square",
    "2:DDS Triangle",
    "3:SPI-A Poll",
    "4:Basic Sweep",
    "5:Single Freq",
    "6:Arb→Basic",
};

#define TEST_COUNT (int)(sizeof(g_test_sequence) / sizeof(g_test_sequence[0]))

// ─────────────────────────────────────────
//  Background test task
// ─────────────────────────────────────────
static TaskHandle_t g_test_task = nullptr;
static volatile bool g_test_running = false;
static volatile bool g_test_stop = false;

static void dds_spi_test_task(void *arg)
{
    (void)arg;
    g_test_running = true;
    g_test_stop = false;
    g_pass_count = 0;
    g_fail_count = 0;

    log_separator("DDS SPI Diagnostic Test Suite");
    log_add("Bitbang mode: %s", DDS_TEST_USE_BITBANG ? "YES (2us half-period)" : "NO");
    log_add("DDS SPI pins: SCLK=33 MOSI=5 MISO=48 CS=32");
    log_add("SPI-A pins:   MOSI=21 MISO=22 SCLK=23 CS=3");
    log_add(" ");
    log_add("PYNQDDS LEDs to observe during tests:");
    log_add("  LED0=arb_mode, LED1=ESP_activity, LED2=arb_playing, LED3=ESP_done");
    log_add(" ");
    log_add("Starting tests in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (int i = 0; i < TEST_COUNT && !g_test_stop; ++i) {
        g_current_test = i;
        uint32_t start_tick = xTaskGetTickCount();

        log_add(" ");
        log_add(">>> Running: %s", g_test_names[i]);
        g_test_sequence[i]();

        uint32_t elapsed_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
        log_add("    Took ~%lums", elapsed_ms);

        // Small delay between tests
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    g_current_test = -1;
    g_test_running = false;

    log_add(" ");
    log_add("═══════════════════════════════════");
    log_add("  TEST SUITE COMPLETE");
    log_add("  PASS: %lu  FAIL: %lu", g_pass_count, g_fail_count);
    log_add("═══════════════════════════════════");
    log_add(" ");
    log_add("Legend: Check LED states on PYNQDDS board during tests.");
    log_add("  If LED2 (arb_playing) stays ON after basic sweep command,");
    log_add("  the arb→basic handover is broken (Root Cause #2).");
    log_add("  If SPI-A poll shows ERR/BFB, SPI link is unstable.");
    log_add("  If DDS echo frames get no ACK, DDS SPI is broken.");

    vTaskDelete(nullptr);
}

// ─────────────────────────────────────────
//  Button callbacks
// ─────────────────────────────────────────
static void dds_spi_start_event_cb(lv_event_t *event)
{
    (void)event;
    if (g_test_running) {
        log_add("Test already running...");
        return;
    }
    if (g_page_active) {
        log_add("Clearing log...");
        g_log_head = 0;
        g_log_count = 0;
        memset(g_log, 0, sizeof(g_log));
        g_pass_count = 0;
        g_fail_count = 0;
        g_current_test = -1;
        g_slot_count = 0;

        log_add("Starting test suite...");
        g_test_stop = false;
        xTaskCreatePinnedToCore(
            dds_spi_test_task, "dds_test", 16384, nullptr, 2, &g_test_task, 1);
    }
}

static void dds_spi_stop_event_cb(lv_event_t *event)
{
    (void)event;
    if (g_test_running) {
        log_add("Stopping test...");
        g_test_stop = true;
    }
}

static void dds_spi_back_event_cb(lv_event_t *event)
{
    (void)event;
    if (g_test_running) {
        g_test_stop = true;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    g_page_active = false;

#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif
    lv_obj_clean(screen);

    // Re-create main page
    extern void create_main_page_reentry(void);
    create_main_page_reentry();
}

// ─────────────────────────────────────────
//  UI Pump (called from SpiLink_UiPump)
// ─────────────────────────────────────────
void dds_spi_test_ui_pump(void)
{
    if (!g_page_active) return;

    // Update link status
    if (g_link_status != nullptr) {
        const char *text = "Link: --";
        uint32_t color = C_YELLOW;
        if (g_test_running) {
            text = "Link: TESTING";
            color = C_BLUE;
        } else if (g_pass_count > 0 && g_fail_count == 0) {
            text = "Link: PASS";
            color = C_GREEN;
        } else if (g_fail_count > 0) {
            text = "Link: FAIL";
            color = C_RED;
        }
        lv_label_set_text(g_link_status, text);
        lv_obj_set_style_text_color(g_link_status, lv_color_hex(color), LV_PART_MAIN);
    }

    // Update pass/fail counters
    if (g_pass_count_label != nullptr) {
        char buf[32];
        snprintf(buf, sizeof(buf), "PASS: %lu", g_pass_count);
        lv_label_set_text(g_pass_count_label, buf);
    }
    if (g_fail_count_label != nullptr) {
        char buf[32];
        snprintf(buf, sizeof(buf), "FAIL: %lu", g_fail_count);
        lv_label_set_text(g_fail_count_label, buf);
    }

    // Update overall
    if (g_overall_label != nullptr) {
        char buf[64];
        if (g_test_running) {
            if (g_current_test >= 0 && g_current_test < TEST_COUNT) {
                snprintf(buf, sizeof(buf), "Running: %s", g_test_names[g_current_test]);
            } else {
                snprintf(buf, sizeof(buf), "Running: ...");
            }
        } else if (g_pass_count + g_fail_count == 0) {
            snprintf(buf, sizeof(buf), "Ready - Press START");
        } else if (g_fail_count == 0) {
            snprintf(buf, sizeof(buf), "ALL PASS (%lu tests)", g_pass_count);
        } else {
            snprintf(buf, sizeof(buf), "FAILURES: %lu/%lu", g_fail_count, g_pass_count + g_fail_count);
        }
        lv_label_set_text(g_overall_label, buf);
    }
}

// ─────────────────────────────────────────
//  Page creation
// ─────────────────────────────────────────
static lv_obj_t *create_label(lv_obj_t *parent, const char *text,
                               int32_t x, int32_t y, int32_t w,
                               const lv_font_t *font, uint32_t color)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_width(l, w);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, lv_color_hex(color), LV_PART_MAIN);
    return l;
}

static lv_obj_t *create_button(lv_obj_t *parent, const char *text,
                                int32_t x, int32_t y, int32_t w)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_width(btn, w);
    lv_obj_t *l = lv_label_create(btn);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return btn;
}

static void create_hline(lv_obj_t *parent, int32_t y)
{
    lv_obj_t *line = lv_line_create(parent);
    lv_obj_set_pos(line, 0, y);
    lv_point_t p[2] = {{0, 0}, {1024, 0}};
    lv_line_set_points(line, p, 2);
    lv_obj_set_style_line_color(line, lv_color_hex(0x334155), LV_PART_MAIN);
    lv_obj_set_style_line_width(line, 1, LV_PART_MAIN);
}

void create_dds_spi_test_page(void)
{
#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_clean(screen);
    g_page_active = true;
    g_log_head = 0;
    g_log_count = 0;
    memset(g_log, 0, sizeof(g_log));
    g_pass_count = 0;
    g_fail_count = 0;
    g_current_test = -1;
    g_test_running = false;

    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(screen, lv_color_hex(C_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    // Title row
    create_label(screen, "DDS SPI Diagnostic", 20, 14, 360, &lv_font_montserrat_24, C_TEXT);
    g_link_status = create_label(screen, "Link: READY", 760, 18, 220, &lv_font_montserrat_18, C_YELLOW);
    create_hline(screen, 52);

    // Legend
    create_label(screen, "PYNQDDS LEDs:", 20, 60, 120, &lv_font_montserrat_14, C_SUB);
    create_label(screen, "LED0=arb_mode  LED1=ESP_activity  LED2=arb_playing  LED3=ESP_done", 20, 78, 700, &lv_font_montserrat_14, C_BLUE);
    create_hline(screen, 100);

    // Buttons row
    lv_obj_t *btn_start = create_button(screen, "START Tests", 20, 108, 140);
    lv_obj_add_event_cb(btn_start, dds_spi_start_event_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *btn_stop = create_button(screen, "STOP", 170, 108, 100);
    lv_obj_add_event_cb(btn_stop, dds_spi_stop_event_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *btn_back = create_button(screen, "Back", 280, 108, 100);
    lv_obj_add_event_cb(btn_back, dds_spi_back_event_cb, LV_EVENT_CLICKED, nullptr);

    // Status labels
    g_pass_count_label = create_label(screen, "PASS: 0", 400, 108, 100, &lv_font_montserrat_16, C_GREEN);
    g_fail_count_label = create_label(screen, "FAIL: 0", 510, 108, 100, &lv_font_montserrat_16, C_RED);
    g_overall_label = create_label(screen, "Ready - Press START", 620, 108, 380, &lv_font_montserrat_16, C_TEXT);

    create_hline(screen, 140);

    // Log area
    g_log_area = lv_textarea_create(screen);
    lv_obj_set_pos(g_log_area, 20, 148);
    lv_obj_set_size(g_log_area, 984, 370);
    lv_textarea_set_text(g_log_area, "");
    lv_textarea_set_font(g_log_area, &lv_font_montserrat_14);
    lv_obj_set_style_text_color(g_log_area, lv_color_hex(C_GREEN), LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_log_area, lv_color_hex(0x0A1520), LV_PART_MAIN);
    lv_obj_set_style_border_color(g_log_area, lv_color_hex(0x334155), LV_PART_MAIN);
    lv_textarea_set_cursor_pos(g_log_area, LV_TEXTAREA_CURSOR_POS_NONE);
    lv_textarea_set_readonly(g_log_area, true);

    log_add("DDS SPI Diagnostic Test Suite");
    log_add("Press START to begin. Observe PYNQDDS LEDs during tests.");
    log_add(" ");
    log_add("Test sequence:");
    log_add("  0: DDS Echo      - ESP→DDS SPI link");
    log_add("  1: DDS Square    - Arbitrary wave output");
    log_add("  2: DDS Triangle   - Arbitrary wave (2nd)");
    log_add("  3: SPI-A Poll     - ESP→PYNQADC SPI link");
    log_add("  4: Basic Sweep    - Full chain sweep");
    log_add("  5: Single Freq    - Full chain single tone");
    log_add("  6: Arb→Basic      - Wave output handover");
}