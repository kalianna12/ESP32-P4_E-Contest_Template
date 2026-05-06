# ESP32-P4 Electric Contest Template

This template was copied from the known-working `esp_brookesia_phone` project
and then reduced in the copy only.

## Layout

```text
esp32p4_elec_workspace/
  common_components/
  esp32p4_elec_template/
```

The project keeps the original dependency shape:

```cmake
set(EXTRA_COMPONENT_DIRS
    ../common_components
)
```

## Kept

- ESP32-P4 Function EV Board BSP copied from the working project
- `../common_components/bsp_extra`
- Display init through the existing MIPI/LVGL adapter
- Minimal LVGL screen
- Wi-Fi STA initialization through ESP-Hosted / Wi-Fi remote
- Audio codec init, muted by default
- Optional SD card API, disabled by default
- Local trimmed copies of BSP, LVGL adapter and bsp_extra to avoid pulling
  camera/video/UVC/SPIFFS/audio-player/Freetype/PNG helper components.

## Removed From Build

- Brookesia phone shell
- Apps component
- AI detect components
- SPIFFS resource image
- Music/video/camera/game/test app startup
- Audio player task
- SD card boot mount unless enabled by Kconfig

## Manual Build Commands

Run from `esp32p4_elec_template` in an ESP-IDF terminal:

```powershell
idf.py --version
where riscv32-esp-elf-gcc
riscv32-esp-elf-gcc --version
idf.py set-target esp32p4
idf.py menuconfig
idf.py build
```

If you still see:

```text
--enable-non-contiguous-regions discards section
```

prefer fixing the ESP-IDF/toolchain pairing first. Do not edit the linker
script or toggle that linker option manually.
