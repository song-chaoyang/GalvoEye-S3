/**
 * @file safety_system.h
 * @brief 安全保护系统 - PIR + ToF + MOSFET 激光电源控制
 *
 * 本文件实现了激光振镜投影仪的多层安全保护系统，包括:
 * - PIR 人体红外传感器: 检测是否有人进入投影区域
 * - VL53L1X ToF 测距传感器: 精确测量人员与激光投影面的距离
 * - MOSFET 激光电源控制: 紧急情况下切断激光供电
 * - 安全状态机: NORMAL -> WARNING -> DANGER -> EMERGENCY_OFF -> RECOVERY_PENDING
 * - FreeRTOS 独立任务: 50Hz 安全检测循环
 * - FreeRTOS 互斥锁: 保护多任务并发访问
 *
 * 安全策略:
 * - 正常模式: 距离 > 2m，激光正常工作
 * - 警告模式: 距离 1-2m，降低激光亮度
 * - 危险模式: 距离 < 1m，关闭激光但保持系统运行
 * - 紧急关断: PIR 检测到近距离人体或手动触发，完全切断激光电源
 * - 恢复待定: 从 DANGER/WARNING/EMERGENCY_OFF 恢复时，需用户确认才回到 NORMAL
 *
 * @note 基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)
 */

#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <Arduino.h>
#include <Wire.h>
#include <freertos/semphr.h>
#include "pin_defs.h"

// 尝试包含 VL53L1X 库
#if __has_include(<VL53L1X.h>)
#include <VL53L1X.h>
#define HAS_VL53L1X 1
#else
#define HAS_VL53L1X 0
#warning "VL53L1X 库未找到，ToF 传感器功能将被禁用"
#endif

// ============================================================
// 安全状态枚举
// ============================================================

enum SafetyState {
    SAFETY_NORMAL = 0,              // 正常状态 - 激光全功率工作
    SAFETY_WARNING = 1,             // 警告状态 - 降低激光亮度
    SAFETY_DANGER = 2,              // 危险状态 - 关闭激光
    SAFETY_EMERGENCY_OFF = 3,       // 紧急关断 - 切断激光电源
    SAFETY_RECOVERY_PENDING = 4     // 恢复待定 - 等待用户确认恢复
};

/**
 * @brief 将安全状态转换为字符串
 */
static inline const char* safetyStateToString(SafetyState state) {
    switch (state) {
        case SAFETY_NORMAL:              return "NORMAL";
        case SAFETY_WARNING:             return "WARNING";
        case SAFETY_DANGER:              return "DANGER";
        case SAFETY_EMERGENCY_OFF:       return "EMERGENCY_OFF";
        case SAFETY_RECOVERY_PENDING:    return "RECOVERY_PENDING";
        default:                         return "UNKNOWN";
    }
}

// ============================================================
// 安全系统类
// ============================================================

class SafetySystem {
public:
    /**
     * @brief 构造函数
     */
    SafetySystem() :
        _currentState(SAFETY_NORMAL),
        _prevState(SAFETY_NORMAL),
        _pirState(false),
        _tofDistance(9999),
        _tofValid(false),
        _manualOverride(false),
        _safetyEnabled(true),
        _emergencyStop(false),
        _initialized(false),
        _lastPIRTrigger(0),
        _warningCount(0),
        _dangerCount(0),
        _stateChangeCallback(nullptr),
        _mutex(nullptr)
#if HAS_VL53L1X
        , _tofSensor(nullptr)
#endif
    {
    }

    /**
     * @brief 析构函数
     */
    ~SafetySystem() {
        // 释放互斥锁
        if (_mutex) {
            vSemaphoreDelete(_mutex);
            _mutex = nullptr;
        }
#if HAS_VL53L1X
        if (_tofSensor) {
            delete _tofSensor;
            _tofSensor = nullptr;
        }
#endif
    }

    /**
     * @brief 初始化安全系统
     * @return true 初始化成功
     */
    bool begin() {
        Serial.println("[安全] 正在初始化安全系统...");

        // --- 创建互斥锁 ---
        _mutex = xSemaphoreCreateMutex();
        if (!_mutex) {
            Serial.println("[安全] !!! 互斥锁创建失败 !!!");
            return false;
        }
        Serial.println("[安全] 互斥锁创建成功");

        // --- 初始化 PIR 传感器 ---
        pinMode(PIN_PIR_SENSOR, INPUT_PULLDOWN);
        _pirState = (digitalRead(PIN_PIR_SENSOR) == HIGH);
        Serial.printf("[安全] PIR 传感器初始状态: %s\n",
                      _pirState ? "检测到人体" : "无人体");

        // --- 初始化 MOSFET 激光电源控制 ---
        pinMode(PIN_MOSFET, OUTPUT);
        // 默认关闭激光电源 (安全优先)
        digitalWrite(PIN_MOSFET, LOW);
        Serial.println("[安全] 激光电源 MOSFET 初始化完成 (默认关闭)");

        // --- 初始化 ToF 传感器 ---
        _tofValid = initToF();
        if (_tofValid) {
            Serial.println("[安全] VL53L1X ToF 传感器初始化成功");
        } else {
            Serial.println("[安全] VL53L1X ToF 传感器初始化失败，将仅使用 PIR 传感器");
        }

        // --- 初始化安全状态 LED ---
        pinMode(PIN_LED_SAFETY, OUTPUT);
        digitalWrite(PIN_LED_SAFETY, LOW);

        // --- 初始安全状态评估 ---
        // 如果 ToF 不可用且 PIR 未检测到人，直接进入 NORMAL 状态并开启激光电源
        if (!_tofValid && !_pirState) {
            _currentState = SAFETY_NORMAL;
            digitalWrite(PIN_MOSFET, HIGH);
            Serial.println("[安全] ToF 不可用且无人体检测，默认进入正常状态，激光电源已开启");
        }

        _initialized = true;
        Serial.println("[安全] 安全系统初始化完成");
        return true;
    }

    /**
     * @brief 启用激光电源
     * @note 仅在安全状态为 NORMAL 时有效
     */
    void enableLaserPower() {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            if (_currentState == SAFETY_NORMAL || _currentState == SAFETY_WARNING) {
                digitalWrite(PIN_MOSFET, HIGH);
                Serial.println("[安全] 激光电源已启用");
            } else {
                Serial.println("[安全] 当前安全状态不允许启用激光电源");
            }
            xSemaphoreRelease(_mutex);
        }
    }

    /**
     * @brief 禁用激光电源
     */
    void disableLaserPower() {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            digitalWrite(PIN_MOSFET, LOW);
            Serial.println("[安全] 激光电源已禁用");
            xSemaphoreRelease(_mutex);
        }
    }

    /**
     * @brief 获取激光电源状态
     * @return true 电源已启用
     */
    bool isLaserPowerEnabled() const {
        return digitalRead(PIN_MOSFET) == HIGH;
    }

    /**
     * @brief 触发紧急停止
     */
    void triggerEmergencyStop() {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _emergencyStop = true;
            _currentState = SAFETY_EMERGENCY_OFF;
            digitalWrite(PIN_MOSFET, LOW);
            Serial.println("[安全] !!! 紧急停止已触发 !!!");
            xSemaphoreRelease(_mutex);
        }
    }

    /**
     * @brief 解除紧急停止
     */
    void clearEmergencyStop() {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _emergencyStop = false;
            // 不直接恢复到 NORMAL，进入恢复待定状态
            _currentState = SAFETY_RECOVERY_PENDING;
            Serial.println("[安全] 紧急停止已解除，进入恢复待定状态");
            xSemaphoreRelease(_mutex);
        }
    }

    /**
     * @brief 检查是否处于紧急停止状态
     */
    bool isEmergencyStopped() const {
        return _emergencyStop;
    }

    /**
     * @brief 设置安全系统启用/禁用
     * @param enabled true=启用, false=禁用 (仅用于调试!)
     */
    void setEnabled(bool enabled) {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _safetyEnabled = enabled;
            if (enabled) {
                Serial.println("[安全] 安全系统已启用");
            } else {
                Serial.println("[安全] !!! 安全系统已禁用 - 仅限调试使用 !!!");
            }
            xSemaphoreRelease(_mutex);
        }
    }

    /**
     * @brief 检查安全系统是否启用
     */
    bool isEnabled() const {
        return _safetyEnabled;
    }

    /**
     * @brief 设置手动覆盖模式 (强制 NORMAL 状态)
     * @param override true=强制正常, false=恢复正常检测
     */
    void setManualOverride(bool override) {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _manualOverride = override;
            if (override) {
                Serial.println("[安全] 手动覆盖模式已启用 - 安全检测被跳过");
            } else {
                Serial.println("[安全] 手动覆盖模式已关闭 - 恢复安全检测");
            }
            xSemaphoreRelease(_mutex);
        }
    }

    /**
     * @brief 获取当前安全状态
     */
    SafetyState getCurrentState() const {
        SafetyState state;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            state = _currentState;
            xSemaphoreRelease(_mutex);
        }
        return state;
    }

    /**
     * @brief 获取 ToF 测距值 (毫米)
     */
    uint16_t getToFDistance() const {
        uint16_t dist;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            dist = _tofDistance;
            xSemaphoreRelease(_mutex);
        }
        return dist;
    }

    /**
     * @brief 检查 ToF 传感器数据是否有效
     */
    bool isToFValid() const {
        return _tofValid;
    }

    /**
     * @brief 获取 PIR 传感器状态
     */
    bool getPIRState() const {
        bool state;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            state = _pirState;
            xSemaphoreRelease(_mutex);
        }
        return state;
    }

    /**
     * @brief 检查激光是否可以安全工作
     * @return true 可以安全工作
     */
    bool isSafeToOperate() const {
        bool safe;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            safe = (_currentState == SAFETY_NORMAL);
            xSemaphoreRelease(_mutex);
        }
        return safe;
    }

    /**
     * @brief 获取建议的激光亮度比例 (0.0 - 1.0)
     * @return 亮度比例
     */
    float getRecommendedBrightness() const {
        float brightness = 0.0f;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            switch (_currentState) {
                case SAFETY_NORMAL:
                    brightness = 1.0f;
                    break;
                case SAFETY_WARNING:
                    // 根据距离线性降低亮度
                    if (_tofDistance >= TOF_WARNING_DISTANCE && _tofDistance <= TOF_SAFE_DISTANCE) {
                        float ratio = (float)(_tofDistance - TOF_WARNING_DISTANCE) /
                                      (float)(TOF_SAFE_DISTANCE - TOF_WARNING_DISTANCE);
                        brightness = 0.3f + 0.7f * ratio; // 最低 30% 亮度
                    } else {
                        brightness = 0.3f;
                    }
                    break;
                case SAFETY_DANGER:
                case SAFETY_EMERGENCY_OFF:
                case SAFETY_RECOVERY_PENDING:
                default:
                    brightness = 0.0f;
                    break;
            }
            xSemaphoreRelease(_mutex);
        }
        return brightness;
    }

    /**
     * @brief 确认安全恢复 - 从 RECOVERY_PENDING 恢复到 NORMAL
     * @return true 恢复成功, false 当前不在恢复待定状态
     *
     * 此方法需要用户主动调用（通过按钮长按或 WebSocket confirmRecovery 指令）
     */
    bool confirmRecovery() {
        bool success = false;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            if (_currentState == SAFETY_RECOVERY_PENDING) {
                // 恢复前再次检查传感器状态，确保环境安全
                SafetyState checkState = evaluateSafetyInternal();
                if (checkState == SAFETY_NORMAL) {
                    setStateInternal(SAFETY_NORMAL);
                    Serial.println("[安全] 用户确认恢复，已回到正常状态");
                    success = true;
                } else {
                    Serial.printf("[安全] 恢复确认失败，当前环境仍不安全: %s\n",
                                  safetyStateToString(checkState));
                    // 保持 RECOVERY_PENDING 状态
                }
            } else {
                Serial.printf("[安全] 当前不在恢复待定状态: %s\n",
                              safetyStateToString(_currentState));
            }
            xSemaphoreRelease(_mutex);
        }
        return success;
    }

    /**
     * @brief 检查是否处于恢复待定状态
     * @return true 处于恢复待定状态
     */
    bool isRecoveryPending() const {
        bool pending;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            pending = (_currentState == SAFETY_RECOVERY_PENDING);
            xSemaphoreRelease(_mutex);
        }
        return pending;
    }

    /**
     * @brief 注册状态变化回调函数
     * @param callback 回调函数 (参数: 新状态, 旧状态)
     */
    typedef void (*StateChangeCallback)(SafetyState newState, SafetyState oldState);
    void onStateChange(StateChangeCallback callback) {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _stateChangeCallback = callback;
            xSemaphoreRelease(_mutex);
        }
    }

    /**
     * @brief 安全检测循环 (在 FreeRTOS 任务中调用)
     *
     * 本函数执行一次完整的安全检测:
     * 1. 读取 PIR 传感器
     * 2. 读取 ToF 测距
     * 3. 根据传感器数据更新安全状态
     * 4. 控制激光电源
     * 5. 更新状态 LED
     */
    void update() {
        if (!_initialized) return;

        if (xSemaphoreTake(_mutex, portMAX_DELAY) != pdTRUE) {
            return; // 获取互斥锁失败，跳过本次检测
        }

        // 如果安全系统被禁用或手动覆盖，跳过检测
        if (!_safetyEnabled || _manualOverride) {
            if (_manualOverride && _currentState != SAFETY_NORMAL) {
                setStateInternal(SAFETY_NORMAL);
            }
            xSemaphoreRelease(_mutex);
            return;
        }

        // 检查紧急停止状态
        if (_emergencyStop) {
            xSemaphoreRelease(_mutex);
            return;
        }

        // 如果当前处于恢复待定状态，不自动恢复，等待用户确认
        if (_currentState == SAFETY_RECOVERY_PENDING) {
            // 仍然读取传感器数据用于状态上报
            bool newPIR = (digitalRead(PIN_PIR_SENSOR) == HIGH);
            if (newPIR != _pirState) {
                _pirState = newPIR;
                _lastPIRTrigger = millis();
                Serial.printf("[安全] PIR 状态变化: %s\n",
                              _pirState ? "检测到人体" : "人体离开");
            }
            if (_tofValid) {
                readToF();
            }
            updateSafetyLED();
            xSemaphoreRelease(_mutex);
            return;
        }

        // --- 步骤 1: 读取 PIR 传感器 ---
        bool newPIR = (digitalRead(PIN_PIR_SENSOR) == HIGH);
        if (newPIR != _pirState) {
            _pirState = newPIR;
            _lastPIRTrigger = millis();
            Serial.printf("[安全] PIR 状态变化: %s\n",
                          _pirState ? "检测到人体" : "人体离开");
        }

        // --- 步骤 2: 读取 ToF 测距 ---
        if (_tofValid) {
            readToF();
        }

        // --- 步骤 3: 根据传感器数据更新安全状态 ---
        SafetyState newState = evaluateSafetyInternal();

        // --- 步骤 4: 执行状态转换 ---
        if (newState != _currentState) {
            // 从非正常状态恢复到正常状态时，进入恢复待定状态
            if ((_prevState == SAFETY_DANGER || _prevState == SAFETY_EMERGENCY_OFF) && newState == SAFETY_NORMAL) {
                setStateInternal(SAFETY_RECOVERY_PENDING);
            } else {
                setStateInternal(newState);
            }
        }

        // --- 步骤 5: 更新状态 LED ---
        updateSafetyLED();

        xSemaphoreRelease(_mutex);
    }

    /**
     * @brief 获取系统状态 JSON 字符串 (用于 WebSocket 上报)
     * @param buffer 输出缓冲区
     * @param bufferSize 缓冲区大小
     */
    void getStatusJSON(char* buffer, size_t bufferSize) const {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            snprintf(buffer, bufferSize,
                     "{\"safety\":{\"state\":\"%s\",\"stateCode\":%d,"
                     "\"tofDistance\":%d,\"tofValid\":%s,"
                     "\"pirDetected\":%s,\"laserPower\":%s,"
                     "\"brightness\":%.2f}}",
                     safetyStateToString(_currentState),
                     (int)_currentState,
                     _tofDistance,
                     _tofValid ? "true" : "false",
                     _pirState ? "true" : "false",
                     isLaserPowerEnabled() ? "true" : "false",
                     getRecommendedBrightness());
            xSemaphoreRelease(_mutex);
        }
    }

private:
    // ========================================================
    // 私有成员变量
    // ========================================================
    SafetyState _currentState;       // 当前安全状态
    SafetyState _prevState;          // 上一次安全状态
    bool _pirState;                  // PIR 传感器当前状态
    uint16_t _tofDistance;           // ToF 测距值 (毫米)
    bool _tofValid;                  // ToF 传感器是否可用
    bool _manualOverride;            // 手动覆盖标志
    bool _safetyEnabled;             // 安全系统启用标志
    bool _emergencyStop;             // 紧急停止标志
    bool _initialized;               // 初始化标志
    unsigned long _lastPIRTrigger;   // 上次 PIR 触发时间
    uint16_t _warningCount;          // 警告计数
    uint16_t _dangerCount;           // 危险计数
    StateChangeCallback _stateChangeCallback; // 状态变化回调
    mutable SemaphoreHandle_t _mutex; // FreeRTOS 互斥锁 (mutable 允许在 const 方法中使用)

#if HAS_VL53L1X
    VL53L1X* _tofSensor;             // ToF 传感器对象
#endif

    // ========================================================
    // 私有方法
    // ========================================================

    /**
     * @brief 初始化 VL53L1X ToF 传感器
     * @return true 初始化成功
     */
    bool initToF() {
#if HAS_VL53L1X
        // 初始化 I2C
        Wire.begin(PIN_TOF_SDA, PIN_TOF_SCL);

        // 创建并初始化传感器
        _tofSensor = new VL53L1X();
        if (!_tofSensor) {
            Serial.println("[安全] ToF 传感器内存分配失败");
            return false;
        }

        _tofSensor->setTimeout(500);
        if (!_tofSensor->init()) {
            Serial.println("[安全] VL53L1X 初始化失败");
            delete _tofSensor;
            _tofSensor = nullptr;
            return false;
        }

        // 配置传感器参数
        _tofSensor->setDistanceMode(VL53L1X::Long);     // 长距离模式
        _tofSensor->setMeasurementTimingBudget(50000);   // 50ms 测量时间
        _tofSensor->startContinuous(20);                 // 20ms 连续测量间隔

        // 读取一次数据验证
        delay(100);
        _tofDistance = _tofSensor->read(false);
        return true;
#else
        return false;
#endif
    }

    /**
     * @brief 读取 ToF 测距数据
     */
    void readToF() {
#if HAS_VL53L1X
        if (!_tofSensor) return;

        uint16_t dist = _tofSensor->read(false);
        if (_tofSensor->timeoutOccurred()) {
            // 超时，保持上一次的有效值
            return;
        }

        // 过滤异常值 (VL53L1X 返回 65535 表示无效)
        if (dist < 65535 && dist > 0) {
            // 简单低通滤波
            _tofDistance = (uint16_t)(_tofDistance * 0.7f + dist * 0.3f);
        }
#endif
    }

    /**
     * @brief 根据传感器数据评估安全状态 (内部方法，调用前需持有互斥锁)
     * @return 评估后的安全状态
     */
    SafetyState evaluateSafetyInternal() {
        // PIR 检测到人体 + ToF 距离很近 -> 紧急关断
        if (_pirState && _tofValid && _tofDistance < TOF_DANGER_DISTANCE) {
            _dangerCount++;
            if (_dangerCount > 5) { // 连续 5 次检测到危险 -> 紧急关断
                return SAFETY_EMERGENCY_OFF;
            }
            return SAFETY_DANGER;
        } else {
            _dangerCount = 0;
        }

        // 仅 ToF 距离判断
        if (_tofValid) {
            if (_tofDistance < TOF_DANGER_DISTANCE) {
                return SAFETY_DANGER;
            } else if (_tofDistance < TOF_WARNING_DISTANCE) {
                return SAFETY_WARNING;
            } else {
                return SAFETY_NORMAL;
            }
        }

        // ToF 不可用，仅使用 PIR
        if (_pirState) {
            // PIR 检测到人体，保守策略: 警告状态
            _warningCount++;
            if (_warningCount > 25) { // 持续 0.5 秒 -> 危险
                return SAFETY_DANGER;
            }
            return SAFETY_WARNING;
        } else {
            _warningCount = 0;
            return SAFETY_NORMAL;
        }
    }

    /**
     * @brief 设置新的安全状态 (内部方法，调用前需持有互斥锁)
     * @param newState 新状态
     */
    void setStateInternal(SafetyState newState) {
        _prevState = _currentState;
        _currentState = newState;

        Serial.printf("[安全] 状态变化: %s -> %s\n",
                      safetyStateToString(_prevState),
                      safetyStateToString(_currentState));

        // 根据新状态控制激光电源
        switch (_currentState) {
            case SAFETY_NORMAL:
                digitalWrite(PIN_MOSFET, HIGH);
                break;

            case SAFETY_WARNING:
                // 警告状态下保持电源开启，但降低亮度
                // (亮度降低由主循环根据 getRecommendedBrightness() 处理)
                digitalWrite(PIN_MOSFET, HIGH);
                break;

            case SAFETY_DANGER:
                // 危险状态: 关闭激光但保持电源 (快速恢复)
                digitalWrite(PIN_MOSFET, LOW);
                break;

            case SAFETY_EMERGENCY_OFF:
                // 紧急关断: 完全切断电源
                digitalWrite(PIN_MOSFET, LOW);
                break;

            case SAFETY_RECOVERY_PENDING:
                // 恢复待定: 关闭激光电源，等待用户确认
                digitalWrite(PIN_MOSFET, LOW);
                break;
        }

        // 触发回调
        if (_stateChangeCallback) {
            _stateChangeCallback(_currentState, _prevState);
        }
    }

    /**
     * @brief 更新安全状态 LED
     */
    void updateSafetyLED() {
        static unsigned long lastBlink = 0;
        static bool ledState = false;
        unsigned long now = millis();

        switch (_currentState) {
            case SAFETY_NORMAL:
                // 常亮
                digitalWrite(PIN_LED_SAFETY, HIGH);
                break;

            case SAFETY_WARNING:
                // 慢闪
                if (now - lastBlink > LED_BLINK_SLOW_MS) {
                    ledState = !ledState;
                    digitalWrite(PIN_LED_SAFETY, ledState ? HIGH : LOW);
                    lastBlink = now;
                }
                break;

            case SAFETY_DANGER:
                // 快闪
                if (now - lastBlink > LED_BLINK_FAST_MS) {
                    ledState = !ledState;
                    digitalWrite(PIN_LED_SAFETY, ledState ? HIGH : LOW);
                    lastBlink = now;
                }
                break;

            case SAFETY_EMERGENCY_OFF:
                // 常灭
                digitalWrite(PIN_LED_SAFETY, LOW);
                break;

            case SAFETY_RECOVERY_PENDING:
                // 交替闪烁 (快闪2次 + 暂停，表示等待确认)
                {
                    static int blinkCount = 0;
                    if (now - lastBlink > 150) {
                        blinkCount++;
                        if (blinkCount <= 4) {
                            ledState = !ledState;
                            digitalWrite(PIN_LED_SAFETY, ledState ? HIGH : LOW);
                        } else if (blinkCount <= 7) {
                            digitalWrite(PIN_LED_SAFETY, LOW);
                        } else {
                            blinkCount = 0;
                        }
                        lastBlink = now;
                    }
                }
                break;
        }
    }
};

// ============================================================
// FreeRTOS 安全检测任务
// ============================================================

/**
 * @brief 安全系统全局指针 (由 main.cpp 设置)
 */
extern SafetySystem* g_safetySystem;

/**
 * @brief FreeRTOS 安全检测任务函数
 * @param param 任务参数 (SafetySystem 指针)
 *
 * 本任务以 50Hz 频率运行安全检测循环:
 * - 读取 PIR 和 ToF 传感器
 * - 评估安全状态
 * - 控制激光电源
 * - 更新状态 LED
 */
void safetyTaskFunction(void* param);

#endif // SAFETY_SYSTEM_H
