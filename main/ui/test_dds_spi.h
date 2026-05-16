#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Diagnostic test page entry point.
void create_dds_spi_test_page(void);

// Periodic UI refresh from the main pump loop.
void dds_spi_test_ui_pump(void);

#ifdef __cplusplus
}
#endif