/**
 * @file safety_system.h
 * @brief 安全保护系统 - PIR + ToF + MOSFET 激光电源控制
 *
 * 本文件实现了激光振镜投影仪的多层安全保护系统，包括:
 * - PIR 人体红外传感器: 检测是否有人进入投影区域
 * - VL53L1X ToF 测距传感器: 精确测量人员与激光投影面的距离
 * - MOSFET 激光电源控制: 紧急情况下切断激光供电
 * - 安全状态机: NORMAL -> WARNING -> DANGER -> EMERGENCY_OFF
 * - FreeRTOS 独立任务: 50Hz 安全检测循环
 *
 * 安全策略:
 * - 正常模式: 距离 > 2m，激光正常工作
 * - 警告模式: 距离 1-2m，降低激光亮度
 * - 危险模式: 距离 < 1m，关闭激光但保持系统运行
 * - 紧急关断: PIR 检测到近距离人体或手动触发，完全切断激光电源
 *
 * @note 基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)
 */

#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <Arduino.h>
#include <Wire.h>
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
    SAFETY_NORMAL = 0,       // 正常状态 - 激光全功率工作
    SAFETY_WARNING = 1,      // 警告状态 - 降低激光亮度
    SAFETY_DANGER = 2,       // 危险状态 - 关闭激光
    SAFETY_EMERGENCY_OFF = 3 // 紧急关断 - 切断激光电源
};

/**
 * @brief 将安全状态转换为字符串
 */
static inline const char* safetyStateToString(SafetyState state) {
    switch (state) {
        case SAFETY_NORMAL:        return "NORMAL";
        case SAFETY_WARNING:       return "WARNING";
        case SAFETY_DANGER:        return "DANGER";
        case SAFETY_EMERGENCY_OFF: return "EMERGENCY_OFF";
        default:                   return "UNKNOWN";
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
        _stateChangeCallback(nullptr)
#if HAS_VL53L1X
        , _tofSensor(nullptr)
#endif
    {
    }

    /**
     * @brief 析构函数
     */
    ~SafetySystem() {
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

        // --- 初始化 PIR 传感器 ---
        pinMode(PIN_PIR_SENSOR, INPUT_PULLDOWN);
        _pirState = (digitalRead(PIN_PIR_SENSOR) == HIGH);
        Serial.printf("[安全] PIR 传感器初始状态: %s\n",
                      _pirState ? "检测到人体" : "无人体");

        // --- 初始化 MOSFET 激光电源控制 ---
        pinMode(PIN_LASER_POWER, OUTPUT);
        // 默认关闭激光电源 (安全优先)
        digitalWrite(PIN_LASER_POWER, LOW);
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

        _initialized = true;
        Serial.println("[安全] 安全系统初始化完成");
        return true;
    }

    /**
     * @brief 启用激光电源
     * @note 仅在安全状态为 NORMAL 时有效
     */
    void enableLaserPower() {
        if (_currentState == SAFETY_NORMAL || _currentState == SAFETY_WARNING) {
            digitalWrite(PIN_LASER_POWER, HIGH);
            Serial.println("[安全] 激光电源已启用");
        } else {
            Serial.println("[安全] 当前安全状态不允许启用激光电源");
        }
    }

    /**
     * @brief 禁用激光电源
     */
    void disableLaserPower() {
        digitalWrite(PIN_LASER_POWER, LOW);
        Serial.println("[安全] 激光电源已禁用");
    }

    /**
     * @brief 获取激光电源状态
     * @return true 电源已启用
     */
    bool isLaserPowerEnabled() const {
        return digitalRead(PIN_LASER_POWER) == HIGH;
    }

    /**
     * @brief 触发紧急停止
     */
    void triggerEmergencyStop() {
        _emergencyStop = true;
        _currentState = SAFETY_EMERGENCY_OFF;
        disableLaserPower();
        Serial.println("[安全] !!! 紧急停止已触发 !!!");
    }

    /**
     * @brief 解除紧急停止
     */
    void clearEmergencyStop() {
        _emergencyStop = false;
        // 不直接恢复到 NORMAL，需要经过安全检测
        _currentState = SAFETY_DANGER;
        Serial.println("[安全] 紧急停止已解除，进入危险状态");
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
        _safetyEnabled = enabled;
        if (enabled) {
            Serial.println("[安全] 安全系统已启用");
        } else {
            Serial.println("[安全] !!! 安全系统已禁用 - 仅限调试使用 !!!");
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
        _manualOverride = override;
        if (override) {
            Serial.println("[安全] 手动覆盖模式已启用 - 安全检测被跳过");
        } else {
            Serial.println("[安全] 手动覆盖模式已关闭 - 恢复安全检测");
        }
    }

    /**
     * @brief 获取当前安全状态
     */
    SafetyState getCurrentState() const {
        return _currentState;
    }

    /**
     * @brief 获取 ToF 测距值 (毫米)
     */
    uint16_t getToFDistance() const {
        return _tofDistance;
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
        return _pirState;
    }

    /**
     * @brief 检查激光是否可以安全工作
     * @return true 可以安全工作
     */
    bool isSafeToOperate() const {
        return _currentState == SAFETY_NORMAL;
    }

    /**
     * @brief 获取建议的激光亮度比例 (0.0 - 1.0)
     * @return 亮度比例
     */
    float getRecommendedBrightness() const {
        switch (_currentState) {
            case SAFETY_NORMAL:
                return 1.0f;
            case SAFETY_WARNING:
                // 根据距离线性降低亮度
                if (_tofDistance >= TOF_WARNING_DISTANCE && _tofDistance <= TOF_SAFE_DISTANCE) {
                    float ratio = (float)(_tofDistance - TOF_WARNING_DISTANCE) /
                                  (float)(TOF_SAFE_DISTANCE - TOF_WARNING_DISTANCE);
                    return 0.3f + 0.7f * ratio; // 最低 30% 亮度
                }
                return 0.3f;
            case SAFETY_DANGER:
            case SAFETY_EMERGENCY_OFF:
            default:
                return 0.0f;
        }
    }

    /**
     * @brief 注册状态变化回调函数
     * @param callback 回调函数 (参数: 新状态, 旧状态)
     */
    typedef void (*StateChangeCallback)(SafetyState newState, SafetyState oldState);
    void onStateChange(StateChangeCallback callback) {
        _stateChangeCallback = callback;
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

        // 如果安全系统被禁用或手动覆盖，跳过检测
        if (!_safetyEnabled || _manualOverride) {
            if (_manualOverride && _currentState != SAFETY_NORMAL) {
                setState(SAFETY_NORMAL);
            }
            return;
        }

        // 检查紧急停止状态
        if (_emergencyStop) {
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
        SafetyState newState = evaluateSafety();

        // --- 步骤 4: 执行状态转换 ---
        if (newState != _currentState) {
            setState(newState);
        }

        // --- 步骤 5: 更新状态 LED ---
        updateSafetyLED();
    }

    /**
     * @brief 获取系统状态 JSON 字符串 (用于 WebSocket 上报)
     * @param buffer 输出缓冲区
     * @param bufferSize 缓冲区大小
     */
    void getStatusJSON(char* buffer, size_t bufferSize) const {
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

        // 配置 ToF 传感器关断引脚
        pinMode(PIN_TOF_XSHUT, OUTPUT);
        digitalWrite(PIN_TOF_XSHUT, LOW);
        delay(10);
        digitalWrite(PIN_TOF_XSHUT, HIGH);
        delay(10);

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
     * @brief 根据传感器数据评估安全状态
     * @return 评估后的安全状态
     */
    SafetyState evaluateSafety() {
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
     * @brief 设置新的安全状态
     * @param newState 新状态
     */
    void setState(SafetyState newState) {
        _prevState = _currentState;
        _currentState = newState;

        Serial.printf("[安全] 状态变化: %s -> %s\n",
                      safetyStateToString(_prevState),
                      safetyStateToString(_currentState));

        // 根据新状态控制激光电源
        switch (_currentState) {
            case SAFETY_NORMAL:
                enableLaserPower();
                break;

            case SAFETY_WARNING:
                // 警告状态下保持电源开启，但降低亮度
                // (亮度降低由主循环根据 getRecommendedBrightness() 处理)
                enableLaserPower();
                break;

            case SAFETY_DANGER:
                // 危险状态: 关闭激光但保持电源 (快速恢复)
                digitalWrite(PIN_LASER_POWER, LOW);
                break;

            case SAFETY_EMERGENCY_OFF:
                // 紧急关断: 完全切断电源
                digitalWrite(PIN_LASER_POWER, LOW);
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
