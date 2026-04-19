/**
 * @file pin_defs.h
 * @brief ESP32-S3-CAM 激光振镜投影仪 - 引脚定义
 *
 * 本文件定义了 bbLaser-S3 项目中所有硬件引脚的映射关系。
 * 基于 ESP32-S3-CAM 开发板，包含 DAC、激光 PWM、安全传感器、
 * SD 卡、按钮和 LED 等外设引脚。
 *
 * @note 基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)
 */

#ifndef PIN_DEFS_H
#define PIN_DEFS_H

// ============================================================
// ESP32-S3-CAM 板载引脚
// ============================================================

// 板载 LED (通常为 GPIO4，红灯)
#define PIN_BOARD_LED         4

// 闪光灯 LED (GPIO33)
#define PIN_FLASH_LED         33

// ============================================================
// MCP4822 DAC SPI 引脚 (控制 X/Y 振镜)
// ============================================================

// SPI 总线引脚 (使用 SPI2 / HSPI)
#define PIN_DAC_SPI_SCK       12   // SPI 时钟
#define PIN_DAC_SPI_MOSI      11   // SPI 主出从入 (数据)
#define PIN_DAC_SPI_MISO      13   // SPI 主入从出 (未使用，但保留)
#define PIN_DAC_SPI_CS        10   // DAC 片选信号

// MCP4822 通道选择通过数据位控制，无需额外引脚
// 通道 A -> X 振镜
// 通道 B -> Y 振镜

// ============================================================
// 激光 PWM 引脚 (RGB 三色激光二极管)
// ============================================================

#define PIN_LASER_R           15   // 红色激光 PWM
#define PIN_LASER_G           16   // 绿色激光 PWM
#define PIN_LASER_B           17   // 蓝色激光 PWM

// 激光 PWM 参数
#define LASER_PWM_FREQ        10000  // PWM 频率: 10kHz (避免可见闪烁)
#define LASER_PWM_RESOLUTION  8      // PWM 分辨率: 8位 (0-255)

// ============================================================
// 安全传感器引脚
// ============================================================

// PIR 人体红外传感器
#define PIN_PIR_SENSOR        6     // PIR 数字输出

// VL53L1X ToF 测距传感器 (I2C 接口)
#define PIN_TOF_SDA           21    // I2C 数据线
#define PIN_TOF_SCL           22    // I2C 时钟线
#define PIN_TOF_XSHUT         3     // ToF 传感器关断引脚

// MOSFET 激光电源控制
#define PIN_LASER_POWER       7     // MOSFET 栅极控制 (高电平=开启)

// ============================================================
// SD 卡引脚 (SPI 模式)
// ============================================================

// 注意: SD 卡使用与 DAC 相同的 SPI 总线，通过不同 CS 区分
#define PIN_SD_CS             5     // SD 卡片选信号
#define PIN_SD_SPI_SCK        12    // 复用 DAC SPI 时钟
#define PIN_SD_SPI_MOSI       11    // 复用 DAC SPI MOSI
#define PIN_SD_SPI_MISO       13    // 复用 DAC SPI MISO

// ============================================================
// 按钮引脚
// ============================================================

#define PIN_BTN_MODE          8     // 模式切换按钮 (WiFi/SD卡播放)
#define PIN_BTN_PAUSE         9     // 暂停/继续按钮
#define PIN_BTN_NEXT          14    // 下一个文件按钮

// 按钮去抖参数
#define BTN_DEBOUNCE_MS       50    // 去抖时间 (毫秒)

// ============================================================
// 状态 LED 引脚
// ============================================================

#define PIN_LED_WIFI          1     // WiFi 连接状态 LED
#define PIN_LED_SAFETY        2     // 安全状态 LED
#define PIN_LED_PLAY          18    // 播放状态 LED

// LED 闪烁模式
#define LED_BLINK_SLOW_MS     1000  // 慢闪间隔 (毫秒)
#define LED_BLINK_FAST_MS     200   // 快闪间隔 (毫秒)

// ============================================================
// 安全系统参数
// ============================================================

// ToF 测距安全阈值 (单位: 毫米)
#define TOF_SAFE_DISTANCE     2000  // 安全距离: 2米
#define TOF_WARNING_DISTANCE  1000  // 警告距离: 1米
#define TOF_DANGER_DISTANCE   500   // 危险距离: 0.5米

// 安全检测循环频率
#define SAFETY_TASK_FREQ_HZ   50    // 50Hz 安全检测循环
#define SAFETY_TASK_STACK_SIZE 4096 // 安全任务栈大小 (字节)

// ============================================================
// DAC 输出参数
// ============================================================

// MCP4822 是 12 位 DAC
#define DAC_MAX_VALUE         4095  // 12位最大值
#define DAC_MID_VALUE         2047  // 12位中间值 (零点)

// 振镜移动速度参数
#define GALVO_MAX_SPEED       50    // 最大移动速度 (单位/毫秒)
#define GALVO_MIN_STEP        1     // 最小步进值

// ============================================================
// ILDA 播放参数
// ============================================================

#define ILDA_DEFAULT_FPS      30    // 默认帧率
#define ILDA_MAX_FPS          60    // 最大帧率
#define ILDA_MIN_FPS          10    // 最小帧率
#define ILDA_POINT_RATE       30000 // ILDA 点速率 (点/秒)

// ============================================================
// WiFi 参数
// ============================================================

// AP 模式默认参数
#define AP_SSID               "bbLaser-S3"
#define AP_PASSWORD           "12345678"  // 最少8位
#define AP_CHANNEL            6
#define AP_MAX_CONN           4

// WebSocket 参数
#define WS_PORT               81    // WebSocket 端口
#define WS_MAX_CLIENTS        2     // 最大客户端连接数

// ============================================================
// 系统参数
// ============================================================

#define SERIAL_BAUD_RATE      115200  // 串口波特率
#define SYSTEM_TICK_MS        1       // 系统主循环最小间隔 (毫秒)

#endif // PIN_DEFS_H
