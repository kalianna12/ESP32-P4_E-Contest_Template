idf_component_register(
    SRCS
        app_main.cpp
        dds_direct_link.cpp
        spilink.cpp
        lvgl_adapter_init.c
        wifi/template_wifi.c
        storage/template_sdcard.c
        ui/heavyfit.cpp
        ui/test_screen.cpp
    INCLUDE_DIRS
        .
        wifi
        storage
        ui
    REQUIRES
        bsp_extra
        esp32_p4_function_ev_board_noglib
        esp_event
        esp_lvgl_adapter
        esp_netif
        esp_wifi
        lvgl__lvgl
        nvs_flash
)

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(
    ${LVGL_LIB}
    PRIVATE
        -DLV_LVGL_H_INCLUDE_SIMPLE
)

if(CONFIG_LV_MEM_CUSTOM)
    target_compile_options(
        ${LVGL_LIB}
        PRIVATE
            "-DLV_MEM_CUSTOM_ALLOC(x)=heap_caps_malloc(x, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)"
    )
endif()
