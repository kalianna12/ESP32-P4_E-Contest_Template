#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t template_sdcard_init(void);
esp_err_t template_sdcard_deinit(void);

#ifdef __cplusplus
}
#endif
