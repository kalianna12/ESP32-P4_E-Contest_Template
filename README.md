# ESP32-P4 Electronic Design Contest Template

本工程是基于 ESP-IDF 的 ESP32-P4 电赛模板工程，用于快速搭建显示、触摸、音频、Wi-Fi、SD 卡等基础功能测试环境。

## 1. 工程说明

当前模板主要包含以下功能：

- ESP32-P4 基础工程框架
- LCD 显示初始化
- 触摸初始化
- LVGL 图形界面支持
- Wi-Fi 初始化
- SD 卡初始化，可通过配置开关启用
- Codec 初始化与静音控制
- 1024 × 600 LCD 屏幕适配

## 2. 开发环境

建议使用以下环境：

- ESP-IDF v5.5.x
- VS Code + ESP-IDF 插件
- ESP32-P4 开发板
- 1024 × 600 LCD 屏幕
- LVGL 9.x

## 3. 编译前注意事项

### 3.1 芯片版本低于 3.3 时需要开启兼容配置

如果使用的 ESP32-P4 芯片版本低于 **v3.3**，需要在 `menuconfig` 中勾选对应兼容选项，否则可能出现启动异常、外设初始化异常或运行不稳定等问题。

进入配置菜单：

```bash
idf.py menuconfig
```

然后进入：

```text
Component config
```

找到与 ESP32-P4 芯片版本兼容相关的配置项，并勾选 **支持低于 v3.3 芯片版本** 的选项。

配置完成后保存退出。

建议重新全量编译：

```bash
idf.py fullclean
idf.py build
```

### 3.2 LVGL 控件配置

如果工程中使用了 LVGL 的按钮、滑块、进度条、开关、下拉框、复选框、文本框、键盘等控件，需要确认以下选项已经开启：

```text
Component config
└── LVGL configuration
    └── Widgets
```

需要启用：

```text
Bar
Button
Check Box
Drop down list
Keyboard
Label
Slider
Switch
Text area
```

如果使用了不同字号的 Montserrat 字体，也需要在 LVGL 字体配置中启用对应字体，例如：

```text
Montserrat 14
Montserrat 16
Montserrat 18
Montserrat 20
Montserrat 24
```

否则可能出现类似下面的编译错误：

```text
lv_font_montserrat_16 was not declared in this scope
lv_slider_create was not declared in this scope
lv_textarea_create was not declared in this scope
```

## 4. 编译和烧录

配置完成后执行：

```bash
idf.py build
```

烧录并打开串口监视器：

```bash
idf.py flash monitor
```

如果修改过 `menuconfig`、组件配置或 CMake 文件，建议执行：

```bash
idf.py fullclean
idf.py reconfigure
idf.py build flash monitor
```

## 5. 目录结构

```text
esp32p4_elec_template/
├── CMakeLists.txt
├── main/
│   ├── app_main.cpp
│   ├── CMakeLists.txt
│   ├── lvgl_adapter_init.c
│   ├── lvgl_adapter_init.h
│   ├── ui/
│   │   ├── test_screen.cpp
│   │   └── test_screen.h
│   ├── wifi/
│   │   └── template_wifi.c
│   └── storage/
│       └── template_sdcard.c
├── components/
├── managed_components/
├── sdkconfig
└── README.md
```

## 6. 常见问题

### 6.1 VS Code 可以跳转定义，但头文件有红色下划线

这通常是 VS Code IntelliSense 或 clangd 没有正确读取 ESP-IDF 的编译参数导致的，不一定是代码错误。

建议先确认工程已经成功生成：

```text
build/compile_commands.json
```

然后重启 clangd 或重新配置工程：

```bash
idf.py reconfigure
```

### 6.2 修改代码后烧录还是旧界面

建议执行：

```bash
idf.py fullclean
idf.py build flash monitor
```

同时确认修改的是当前工程路径下的文件，尤其是：

```text
main/app_main.cpp
main/CMakeLists.txt
```

### 6.3 LVGL 控件函数找不到

如果出现：

```text
lv_slider_create was not declared in this scope
lv_bar_create was not declared in this scope
lv_switch_create was not declared in this scope
```

说明对应 LVGL 控件没有在 `menuconfig` 中启用。

进入：

```text
Component config
└── LVGL configuration
    └── Widgets
```

打开对应控件后重新编译。

## 7. 备注

本模板主要用于电赛前期工程验证和外设测试，正式比赛工程可以在此基础上继续添加传感器、控制算法、通信协议和完整 UI 页面。
