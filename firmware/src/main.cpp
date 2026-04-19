/**
 * @file main.cpp
 * @brief GalvoEye-S3 激光振镜投影仪 - 主程序入口
 *
 * 本文件是 GalvoEye-S3 固件的主程序，负责:
 * - 系统初始化 (硬件、外设、网络)
 * - WiFi 连接管理 (AP 模式和 STA 模式)
 * - WebSocket 服务器 (接收 PC 端坐标指令)
 * - SD 卡 ILDA 文件播放
 * - 按钮控制 (切换/暂停)
 * - 安全保护系统集成
 * - DAC 振镜控制
 * - 激光 PWM 控制 (RGB 三色)
 * - 系统状态 LED 指示
 * - FreeRTOS 多任务管理
 *
 * 系统架构:
 *   主循环 (loop):
 *     - WebSocket 通信处理
 *     - ILDA 播放更新
 *     - DAC 位置更新
 *     - 按钮扫描
 *     - LED 状态更新
 *
 *   FreeRTOS 任务:
 *     - safetyTask: 50Hz 安全检测循环 (独立核心)
 *
 * @note 基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)
 * @note 目标平台: ESP32-S3-CAM
 */

#include <Arduino.h>
#include <WiFi.h>
#include "pin_defs.h"
#include "dac_controller.h"
#include "safety_system.h"
#include "websocket_handler.h"
#include "ilda_player.h"

// ============================================================
// 全局对象
// ============================================================

// DAC 控制器 (MCP4822 双通道，控制 X/Y 振镜)
DACController g_dac;

// 安全系统 (PIR + ToF + MOSFET)
SafetySystem g_safety;

// 安全系统全局指针 (供 safety_system.h 中的 FreeRTOS 任务使用)
SafetySystem* g_safetySystem = &g_safety;

// WebSocket 通信处理器
WebSocketHandler g_websocket(WS_PORT);

// ILDA 文件播放器
ILDAPlayer g_ildaPlayer;

// ============================================================
// WiFi 配置
// ============================================================

// STA 模式配置 (连接到现有 WiFi 网络)
// 如果以下 SSID 为空，则使用 AP 模式
#define WIFI_STA_SSID       ""
#define WIFI_STA_PASSWORD   ""

// ============================================================
// 系统运行状态
// ============================================================

// 运行模式
enum SystemMode {
    MODE_WIFI = 0,       // WiFi 实时控制模式
    MODE_ILDA = 1        // ILDA 文件播放模式
};

static SystemMode g_systemMode = MODE_WIFI;   // 当前运行模式
static bool g_wifiConnected = false;          // WiFi 连接状态
static unsigned long g_lastStatusReport = 0;  // 上次状态上报时间
static TaskHandle_t g_safetyTaskHandle = nullptr; // 安全任务句柄

// ============================================================
// 按钮去抖辅助
// ============================================================

static unsigned long g_lastBtnPress[3] = {0, 0, 0}; // 三个按钮的上次按下时间
static const uint8_t g_btnPins[] = {PIN_BTN_MODE, PIN_BTN_PAUSE, PIN_BTN_NEXT};
static const int g_btnCount = 3;

// ============================================================
// LED 状态管理
// ============================================================

static unsigned long g_lastWiFiBlink = 0;
static bool g_wifiLedState = false;
static unsigned long g_lastPlayBlink = 0;
static bool g_playLedState = false;

// ============================================================
// 函数声明
// ============================================================

// 初始化函数
static bool initHardware();
static bool initWiFi();
static bool initSystem();

// WiFi 事件处理
static void onWiFiEvent(WiFiEvent_t event);

// 按钮处理
static void scanButtons();
static void handleModeButton();
static void handlePauseButton();
static void handleNextButton();

// LED 更新
static void updateLEDs();

// 安全状态变化回调
static void onSafetyStateChange(SafetyState newState, SafetyState oldState);

// ILDA 播放控制回调
static void onPlayILDA(const char* filename);
static void onStopILDA();

// FreeRTOS 安全任务
void safetyTaskFunction(void* param);

// ============================================================
// Arduino setup() - 系统初始化
// ============================================================

void setup() {
    // --- 初始化串口 ---
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500); // 等待串口稳定

    Serial.println("");
    Serial.println("========================================");
    Serial.println("  GalvoEye-S3 激光振镜投影仪");
    Serial.println("  基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)");
    Serial.println("  目标平台: ESP32-S3-CAM");
    Serial.println("========================================");
    Serial.printf("  系统启动时间: %lu ms\n", millis());
    Serial.printf("  可用堆内存: %lu 字节\n", ESP.getFreeHeap());
    Serial.printf("  CPU 频率: %lu MHz\n", getCpuFrequencyMhz());
    Serial.println("========================================\n");

    // --- 初始化硬件 ---
    if (!initHardware()) {
        Serial.println("[系统] 硬件初始化失败! 系统停止");
        while (1) {
            // 错误指示: 快速闪烁板载 LED
            digitalWrite(PIN_BOARD_LED, !digitalRead(PIN_BOARD_LED));
            delay(100);
        }
    }

    // --- 初始化 WiFi ---
    initWiFi();

    // --- 初始化系统组件 ---
    if (!initSystem()) {
        Serial.println("[系统] 系统组件初始化失败!");
        // 非致命错误，继续运行 (部分功能可能不可用)
    }

    // --- 创建 FreeRTOS 安全检测任务 ---
    // 在核心 1 上运行 (核心 0 运行 WiFi，核心 1 运行应用)
    BaseType_t result = xTaskCreatePinnedToCore(
        safetyTaskFunction,          // 任务函数
        "safety_task",               // 任务名称
        SAFETY_TASK_STACK_SIZE,      // 栈大小
        nullptr,                     // 参数
        2,                           // 优先级 (较高)
        &g_safetyTaskHandle,         // 任务句柄
        1                            // 核心 1
    );

    if (result != pdPASS) {
        Serial.println("[系统] !!! 安全检测任务创建失败 !!!");
        Serial.println("[系统] 安全系统将无法正常运行!");
    } else {
        Serial.println("[系统] 安全检测任务已启动 (核心1, 50Hz)");
    }

    // --- 启动指示 ---
    Serial.println("\n========================================");
    Serial.println("  系统初始化完成!");
    Serial.printf("  运行模式: %s\n", g_systemMode == MODE_WIFI ? "WiFi 控制" : "ILDA 播放");
    Serial.printf("  WiFi 状态: %s\n", g_wifiConnected ? "已连接" : "未连接");
    Serial.printf("  安全状态: %s\n", safetyStateToString(g_safety.getCurrentState()));
    Serial.printf("  可用堆内存: %lu 字节\n", ESP.getFreeHeap());
    Serial.println("========================================\n");

    // 开机动画: 绘制一个小圆
    if (g_safety.isSafeToOperate()) {
        g_safety.enableLaserPower();
        delay(100);
        g_dac.drawCircle(DAC_MID_VALUE, DAC_MID_VALUE, 300, 0, 255, 0, 32);
        delay(500);
        g_dac.laserOff();
        g_dac.home();
    }
}

// ============================================================
// Arduino loop() - 主循环
// ============================================================

void loop() {
    // --- 1. WebSocket 通信处理 ---
    g_websocket.loop();

    // --- 2. ILDA 播放更新 ---
    if (g_systemMode == MODE_ILDA) {
        // 仅在安全状态允许时播放
        if (g_safety.isSafeToOperate() || g_safety.getCurrentState() == SAFETY_WARNING) {
            g_ildaPlayer.update();

            // 根据安全状态调整亮度
            float brightness = g_safety.getRecommendedBrightness();
            if (brightness < 1.0f) {
                uint8_t b = (uint8_t)(brightness * 255.0f);
                // 注意: 这里仅影响后续绘制的亮度
                // ILDA 播放器内部的亮度控制需要在 renderFrame 中处理
            }
        } else {
            // 安全状态不允许操作，暂停播放
            if (g_ildaPlayer.getState() == ILDA_PLAYING) {
                g_ildaPlayer.pause();
                Serial.println("[系统] 安全状态不允许操作，ILDA 播放已暂停");
            }
        }
    }

    // --- 3. DAC 位置更新 (WiFi 模式下) ---
    if (g_systemMode == MODE_WIFI) {
        if (g_safety.isSafeToOperate()) {
            g_dac.update(0.3f);
        }
    }

    // --- 4. 按钮扫描 ---
    scanButtons();

    // --- 5. LED 状态更新 ---
    updateLEDs();

    // --- 6. 定期状态上报 (每 5 秒) ---
    unsigned long now = millis();
    if (now - g_lastStatusReport > 5000) {
        g_lastStatusReport = now;

        if (g_websocket.isClientConnected()) {
            g_websocket.sendStatus();
        }

        // 串口状态输出
        Serial.printf("[状态] 模式:%s WiFi:%s 安全:%s 堆:%lu\n",
                      g_systemMode == MODE_WIFI ? "WiFi" : "ILDA",
                      g_wifiConnected ? "连接" : "断开",
                      safetyStateToString(g_safety.getCurrentState()),
                      ESP.getFreeHeap());
    }

    // --- 7. 短暂延时，避免 CPU 占用过高 ---
    delay(SYSTEM_TICK_MS);
}

// ============================================================
// 硬件初始化
// ============================================================

/**
 * @brief 初始化所有硬件外设
 * @return true 初始化成功
 */
static bool initHardware() {
    Serial.println("[硬件] 正在初始化硬件...");

    // --- 板载 LED ---
    pinMode(PIN_BOARD_LED, OUTPUT);
    digitalWrite(PIN_BOARD_LED, LOW);
    Serial.println("[硬件] 板载 LED 初始化完成");

    // --- 状态 LED ---
    pinMode(PIN_LED_WIFI, OUTPUT);
    pinMode(PIN_LED_SAFETY, OUTPUT);
    pinMode(PIN_LED_PLAY, OUTPUT);
    digitalWrite(PIN_LED_WIFI, LOW);
    digitalWrite(PIN_LED_SAFETY, LOW);
    digitalWrite(PIN_LED_PLAY, LOW);
    Serial.println("[硬件] 状态 LED 初始化完成");

    // --- 按钮输入 ---
    for (int i = 0; i < g_btnCount; i++) {
        pinMode(g_btnPins[i], INPUT_PULLUP);
    }
    Serial.println("[硬件] 按钮初始化完成");

    // --- DAC 控制器 (MCP4822) ---
    if (!g_dac.begin()) {
        Serial.println("[硬件] DAC 初始化失败!");
        return false;
    }

    // --- 安全系统 (PIR + ToF + MOSFET) ---
    if (!g_safety.begin()) {
        Serial.println("[硬件] 安全系统初始化失败!");
        // 非致命错误，继续运行
    }

    // 注册安全状态变化回调
    g_safety.onStateChange(onSafetyStateChange);

    Serial.println("[硬件] 硬件初始化完成");
    return true;
}

// ============================================================
// WiFi 初始化
// ============================================================

/**
 * @brief 初始化 WiFi 连接
 * @return true WiFi 连接成功
 */
static bool initWiFi() {
    Serial.println("[WiFi] 正在初始化 WiFi...");

    // 配置 WiFi 模式指示 LED
    pinMode(PIN_LED_WIFI, OUTPUT);
    digitalWrite(PIN_LED_WIFI, LOW);

    // 检查是否配置了 STA 模式的 SSID
    if (strlen(WIFI_STA_SSID) > 0) {
        // --- STA 模式: 连接到现有 WiFi 网络 ---
        Serial.printf("[WiFi] 正在连接到: %s\n", WIFI_STA_SSID);

        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);

        // 注册 WiFi 事件回调
        WiFi.onEvent(onWiFiEvent);

        WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);

        // 等待连接 (最多 15 秒)
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 30) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        Serial.println("");

        if (WiFi.status() == WL_CONNECTED) {
            g_wifiConnected = true;
            Serial.printf("[WiFi] 已连接到: %s\n", WIFI_STA_SSID);
            Serial.printf("[WiFi] IP 地址: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("[WiFi] 信号强度: %d dBm\n", WiFi.RSSI());
            digitalWrite(PIN_LED_WIFI, HIGH);
        } else {
            Serial.println("[WiFi] STA 模式连接失败，切换到 AP 模式");
            g_wifiConnected = false;
            // 继续尝试 AP 模式
        }
    }

    if (!g_wifiConnected) {
        // --- AP 模式: 创建 WiFi 热点 ---
        Serial.println("[WiFi] 正在启动 AP 模式...");
        WiFi.mode(WIFI_AP);

        bool apResult = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONN);
        if (apResult) {
            g_wifiConnected = true;
            Serial.printf("[WiFi] AP 已启动\n");
            Serial.printf("[WiFi] SSID: %s\n", AP_SSID);
            Serial.printf("[WiFi] 密码: %s\n", AP_PASSWORD);
            Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
            Serial.printf("[WiFi] 信道: %d\n", AP_CHANNEL);
            digitalWrite(PIN_LED_WIFI, HIGH);
        } else {
            Serial.println("[WiFi] AP 启动失败!");
            g_wifiConnected = false;
        }
    }

    return g_wifiConnected;
}

/**
 * @brief WiFi 事件处理回调
 * @param event WiFi 事件类型
 */
static void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.printf("[WiFi] 获取 IP: %s\n", WiFi.localIP().toString().c_str());
            g_wifiConnected = true;
            digitalWrite(PIN_LED_WIFI, HIGH);
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("[WiFi] 连接断开，正在重连...");
            g_wifiConnected = false;
            digitalWrite(PIN_LED_WIFI, LOW);
            WiFi.reconnect();
            break;

        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("[WiFi] 客户端已连接到 AP");
            break;

        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("[WiFi] 客户端已从 AP 断开");
            break;

        default:
            break;
    }
}

// ============================================================
// 系统组件初始化
// ============================================================

/**
 * @brief 初始化系统组件 (WebSocket、ILDA 播放器)
 * @return true 初始化成功
 */
static bool initSystem() {
    Serial.println("[系统] 正在初始化系统组件...");

    // --- WebSocket 服务器 ---
    g_websocket.setDAC(&g_dac);
    g_websocket.setSafety(&g_safety);
    g_websocket.onPlayILDA(onPlayILDA);
    g_websocket.onStopILDA(onStopILDA);

    if (!g_websocket.begin()) {
        Serial.println("[系统] WebSocket 服务器启动失败!");
        return false;
    }
    Serial.printf("[系统] WebSocket 服务器已启动，端口: %d\n", WS_PORT);

    // --- ILDA 播放器 ---
    g_ildaPlayer.setDAC(&g_dac);
    if (!g_ildaPlayer.begin()) {
        Serial.println("[系统] ILDA 播放器 SD 卡初始化失败");
        Serial.println("[系统] ILDA 文件播放功能不可用");
        // 非致命错误，WiFi 模式仍可使用
    } else {
        int fileCount = g_ildaPlayer.getFileCount();
        Serial.printf("[系统] ILDA 播放器就绪，找到 %d 个文件\n", fileCount);

        // 如果有 ILDA 文件且没有配置 WiFi SSID，默认进入 ILDA 播放模式
        if (fileCount > 0 && strlen(WIFI_STA_SSID) == 0) {
            g_systemMode = MODE_ILDA;
            Serial.println("[系统] 检测到 ILDA 文件，默认进入播放模式");
        }
    }

    Serial.println("[系统] 系统组件初始化完成");
    return true;
}

// ============================================================
// 按钮处理
// ============================================================

/**
 * @brief 扫描所有按钮
 */
static void scanButtons() {
    for (int i = 0; i < g_btnCount; i++) {
        // 按钮使用上拉电阻，按下时为 LOW
        if (digitalRead(g_btnPins[i]) == LOW) {
            unsigned long now = millis();
            if (now - g_lastBtnPress[i] > BTN_DEBOUNCE_MS) {
                g_lastBtnPress[i] = now;

                // 根据按钮索引处理
                switch (i) {
                    case 0: handleModeButton(); break;
                    case 1: handlePauseButton(); break;
                    case 2: handleNextButton();  break;
                }
            }
        }
    }
}

/**
 * @brief 模式切换按钮处理
 *
 * 在 WiFi 控制模式和 ILDA 文件播放模式之间切换
 */
static void handleModeButton() {
    Serial.println("[按钮] 模式切换按钮按下");

    // 停止当前模式
    if (g_systemMode == MODE_ILDA) {
        g_ildaPlayer.stop();
    }

    // 切换模式
    g_systemMode = (g_systemMode == MODE_WIFI) ? MODE_ILDA : MODE_WIFI;

    Serial.printf("[按钮] 当前模式: %s\n",
                  g_systemMode == MODE_WIFI ? "WiFi 控制" : "ILDA 播放");

    // 如果切换到 ILDA 模式，开始播放
    if (g_systemMode == MODE_ILDA) {
        if (g_ildaPlayer.getFileCount() > 0) {
            if (g_safety.isSafeToOperate()) {
                g_safety.enableLaserPower();
                delay(50);
                g_ildaPlayer.play();
            } else {
                Serial.println("[按钮] 安全状态不允许播放");
            }
        } else {
            Serial.println("[按钮] 没有 ILDA 文件可播放");
        }
    }

    // 如果切换到 WiFi 模式，回到中心位置
    if (g_systemMode == MODE_WIFI) {
        g_dac.laserOff();
        g_dac.home();
    }
}

/**
 * @brief 暂停/继续按钮处理
 */
static void handlePauseButton() {
    Serial.println("[按钮] 暂停/继续按钮按下");

    if (g_systemMode == MODE_ILDA) {
        ILDAPlayState state = g_ildaPlayer.getState();
        if (state == ILDA_PLAYING) {
            g_ildaPlayer.pause();
        } else if (state == ILDA_PAUSED) {
            if (g_safety.isSafeToOperate()) {
                g_ildaPlayer.resume();
            } else {
                Serial.println("[按钮] 安全状态不允许恢复播放");
            }
        }
    }
}

/**
 * @brief 下一个文件按钮处理
 */
static void handleNextButton() {
    Serial.println("[按钮] 下一个文件按钮按下");

    if (g_systemMode == MODE_ILDA) {
        if (g_safety.isSafeToOperate()) {
            g_ildaPlayer.playNext();
        } else {
            Serial.println("[按钮] 安全状态不允许切换文件");
        }
    }
}

// ============================================================
// LED 状态更新
// ============================================================

/**
 * @brief 更新所有状态 LED
 */
static void updateLEDs() {
    unsigned long now = millis();

    // --- WiFi LED ---
    if (g_wifiConnected) {
        // 已连接: 常亮
        digitalWrite(PIN_LED_WIFI, HIGH);
    } else {
        // 未连接: 慢闪
        if (now - g_lastWiFiBlink > LED_BLINK_SLOW_MS) {
            g_wifiLedState = !g_wifiLedState;
            digitalWrite(PIN_LED_WIFI, g_wifiLedState ? HIGH : LOW);
            g_lastWiFiBlink = now;
        }
    }

    // --- 安全 LED (由安全系统自行管理) ---
    // 已在 SafetySystem::updateSafetyLED() 中处理

    // --- 播放 LED ---
    if (g_systemMode == MODE_ILDA) {
        ILDAPlayState state = g_ildaPlayer.getState();
        switch (state) {
            case ILDA_PLAYING:
                // 播放中: 常亮
                digitalWrite(PIN_LED_PLAY, HIGH);
                break;

            case ILDA_PAUSED:
                // 暂停: 慢闪
                if (now - g_lastPlayBlink > LED_BLINK_SLOW_MS) {
                    g_playLedState = !g_playLedState;
                    digitalWrite(PIN_LED_PLAY, g_playLedState ? HIGH : LOW);
                    g_lastPlayBlink = now;
                }
                break;

            case ILDA_ERROR:
                // 错误: 快闪
                if (now - g_lastPlayBlink > LED_BLINK_FAST_MS) {
                    g_playLedState = !g_playLedState;
                    digitalWrite(PIN_LED_PLAY, g_playLedState ? HIGH : LOW);
                    g_lastPlayBlink = now;
                }
                break;

            default:
                // 空闲/停止: 灭
                digitalWrite(PIN_LED_PLAY, LOW);
                break;
        }
    } else {
        // WiFi 模式: 灭
        digitalWrite(PIN_LED_PLAY, LOW);
    }
}

// ============================================================
// 回调函数
// ============================================================

/**
 * @brief 安全状态变化回调
 * @param newState 新状态
 * @param oldState 旧状态
 */
static void onSafetyStateChange(SafetyState newState, SafetyState oldState) {
    Serial.printf("[回调] 安全状态变化: %s -> %s\n",
                  safetyStateToString(oldState),
                  safetyStateToString(newState));

    // 通知 WebSocket 客户端
    if (g_websocket.isClientConnected()) {
        char buffer[256];
        g_safety.getStatusJSON(buffer, sizeof(buffer));
        g_websocket.broadcast(buffer);
    }

    // 根据状态变化采取行动
    switch (newState) {
        case SAFETY_NORMAL:
            // 恢复正常: 重新启用激光电源
            g_safety.enableLaserPower();
            // 如果 ILDA 播放被暂停，自动恢复
            if (g_systemMode == MODE_ILDA &&
                g_ildaPlayer.getState() == ILDA_PAUSED) {
                g_ildaPlayer.resume();
            }
            break;

        case SAFETY_WARNING:
            // 警告: 保持运行但降低亮度
            // (亮度由 DAC 控制器在绘制时查询)
            break;

        case SAFETY_DANGER:
            // 危险: 关闭激光
            g_dac.laserOff();
            if (g_systemMode == MODE_ILDA &&
                g_ildaPlayer.getState() == ILDA_PLAYING) {
                g_ildaPlayer.pause();
            }
            break;

        case SAFETY_EMERGENCY_OFF:
            // 紧急关断: 立即关闭一切
            g_dac.laserOff();
            g_dac.home();
            if (g_ildaPlayer.getState() == ILDA_PLAYING ||
                g_ildaPlayer.getState() == ILDA_PAUSED) {
                g_ildaPlayer.stop();
            }
            break;
    }
}

/**
 * @brief ILDA 播放回调 (由 WebSocket 指令触发)
 * @param filename 要播放的文件名
 */
static void onPlayILDA(const char* filename) {
    Serial.printf("[回调] 收到 ILDA 播放指令: %s\n", filename);

    if (!g_safety.isSafeToOperate()) {
        Serial.println("[回调] 安全状态不允许播放");
        g_websocket.sendError("安全状态不允许播放 ILDA 文件");
        return;
    }

    // 切换到 ILDA 模式
    g_systemMode = MODE_ILDA;

    // 启用激光电源
    g_safety.enableLaserPower();
    delay(50);

    // 开始播放
    if (!g_ildaPlayer.play(filename)) {
        char errorMsg[128];
        snprintf(errorMsg, sizeof(errorMsg), "ILDA 播放失败: %s", g_ildaPlayer.getLastError());
        g_websocket.sendError(errorMsg);
    }
}

/**
 * @brief ILDA 停止回调 (由 WebSocket 指令触发)
 */
static void onStopILDA() {
    Serial.println("[回调] 收到 ILDA 停止指令");
    g_ildaPlayer.stop();
    g_dac.home();
}

// ============================================================
// FreeRTOS 安全检测任务
// ============================================================

/**
 * @brief FreeRTOS 安全检测任务函数
 * @param param 任务参数 (未使用)
 *
 * 本任务在核心 1 上以 50Hz 频率独立运行安全检测循环。
 * 使用 vTaskDelay 精确控制检测频率，确保安全系统的实时性。
 *
 * 任务优先级: 2 (较高)
 * 栈大小: 4096 字节
 * 运行核心: 1
 */
void safetyTaskFunction(void* param) {
    Serial.println("[安全任务] 已启动");

    // 计算任务延时 (50Hz = 20ms 周期)
    const TickType_t taskDelay = pdMS_TO_TICKS(1000 / SAFETY_TASK_FREQ_HZ);

    // 任务主循环
    while (true) {
        // 执行安全检测
        if (g_safetySystem) {
            g_safetySystem->update();
        }

        // 精确延时，保持 50Hz 频率
        vTaskDelay(taskDelay);
    }

    // 任务不应到达此处
    vTaskDelete(nullptr);
}
