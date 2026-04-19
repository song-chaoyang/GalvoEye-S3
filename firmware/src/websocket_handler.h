/**
 * @file websocket_handler.h
 * @brief WebSocket 通信处理器 - PC 端指令接收与状态上报
 *
 * 本文件实现了基于 WebSockets 库的通信处理器，负责:
 * - 接收 PC 端发送的目标坐标指令
 * - 接收绘图指令 (点、线、圆、矩形、文字)
 * - 接收安全模式切换指令
 * - 接收 ILDA 播放控制指令
 * - 发送设备状态信息回 PC 端
 * - 客户端连接/断开管理
 *
 * 通信协议 (JSON 格式):
 * 接收指令格式:
 *   {"cmd":"moveTo", "x":2048, "y":2048}         - 移动到坐标
 *   {"cmd":"drawPoint", "x":1000, "y":1000, "r":255, "g":0, "b":0} - 绘制点
 *   {"cmd":"drawLine", "x1":0, "y1":0, "x2":4095, "y2":4095, "r":255, "g":255, "b":255} - 绘制线
 *   {"cmd":"drawCircle", "cx":2048, "cy":2048, "radius":500, "r":0, "g":255, "b":0} - 绘制圆
 *   {"cmd":"drawRect", "x":500, "y":500, "w":3000, "h":3000, "r":0, "g":0, "b":255} - 绘制矩形
 *   {"cmd":"drawText", "text":"Hello", "x":100, "y":100, "scale":3} - 绘制文字
 *   {"cmd":"laserOff"}                              - 关闭激光
 *   {"cmd":"laserOn", "r":255, "g":255, "b":255}   - 开启激光
 *   {"cmd":"setSafety", "mode":"normal|override|off"} - 设置安全模式
 *   {"cmd":"getStatus"}                             - 请求状态
 *   {"cmd":"playILDA", "file":"test.ild"}           - 播放 ILDA 文件
 *   {"cmd":"stopILDA"}                              - 停止 ILDA 播放
 *   {"cmd":"home"}                                  - 回到中心位置
 *
 * @note 基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)
 */

#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "pin_defs.h"

// 前向声明 (避免头文件循环依赖)
class DACController;
class SafetySystem;

// ============================================================
// WebSocket 消息缓冲区大小
// ============================================================

#define WS_RX_BUFFER_SIZE    512   // 接收缓冲区大小
#define WS_TX_BUFFER_SIZE    512   // 发送缓冲区大小
#define WS_MAX_JSON_DEPTH    10    // JSON 解析最大深度

// ============================================================
// WebSocket 事件回调类型
// ============================================================

/**
 * @brief ILDA 播放控制回调类型
 * @param filename 要播放的文件名
 */
typedef void (*PlayILDACallback)(const char* filename);

/**
 * @brief ILDA 停止回调类型
 */
typedef void (*StopILDACallback)();

// ============================================================
// WebSocket 处理器类
// ============================================================

class WebSocketHandler {
public:
    /**
     * @brief 构造函数
     * @param port WebSocket 端口
     */
    WebSocketHandler(uint16_t port = WS_PORT) :
        _server(port),
        _dac(nullptr),
        _safety(nullptr),
        _playILDACallback(nullptr),
        _stopILDACallback(nullptr),
        _clientConnected(false)
    {
    }

    /**
     * @brief 初始化 WebSocket 服务器
     * @return true 初始化成功
     */
    bool begin() {
        // 注册事件处理
        _server.onEvent(
            [this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
                this->handleEvent(num, type, payload, length);
            }
        );

        _server.begin();
        Serial.printf("[WebSocket] 服务器已启动，端口: %d\n", WS_PORT);
        return true;
    }

    /**
     * @brief 设置 DAC 控制器引用
     * @param dac DAC 控制器指针
     */
    void setDAC(DACController* dac) {
        _dac = dac;
    }

    /**
     * @brief 设置安全系统引用
     * @param safety 安全系统指针
     */
    void setSafety(SafetySystem* safety) {
        _safety = safety;
    }

    /**
     * @brief 注册 ILDA 播放回调
     * @param callback 回调函数
     */
    void onPlayILDA(PlayILDACallback callback) {
        _playILDACallback = callback;
    }

    /**
     * @brief 注册 ILDA 停止回调
     * @param callback 回调函数
     */
    void onStopILDA(StopILDACallback callback) {
        _stopILDACallback = callback;
    }

    /**
     * @brief 循环处理 (在主循环中调用)
     */
    void loop() {
        _server.loop();
    }

    /**
     * @brief 检查是否有客户端连接
     * @return true 有客户端连接
     */
    bool isClientConnected() const {
        return _clientConnected;
    }

    /**
     * @brief 向所有连接的客户端广播消息
     * @param message 消息字符串
     */
    void broadcast(const char* message) {
        _server.broadcastTXT(message);
    }

    /**
     * @brief 向指定客户端发送消息
     * @param num 客户端编号
     * @param message 消息字符串
     */
    void send(uint8_t num, const char* message) {
        _server.sendTXT(num, message);
    }

    /**
     * @brief 发送设备状态信息
     */
    void sendStatus() {
        char buffer[WS_TX_BUFFER_SIZE];

        // 基本状态信息
        snprintf(buffer, sizeof(buffer),
                 "{\"type\":\"status\","
                 "\"connected\":true,"
                 "\"freeHeap\":%lu,"
                 "\"uptime\":%lu}",
                 ESP.getFreeHeap(),
                 millis() / 1000);

        broadcast(buffer);

        // 安全状态信息
        if (_safety) {
            char safetyBuffer[256];
            _safety->getStatusJSON(safetyBuffer, sizeof(safetyBuffer));
            broadcast(safetyBuffer);
        }
    }

    /**
     * @brief 发送错误消息
     * @param errorMsg 错误描述
     */
    void sendError(const char* errorMsg) {
        char buffer[WS_TX_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer),
                 "{\"type\":\"error\",\"message\":\"%s\"}", errorMsg);
        broadcast(buffer);
    }

    /**
     * @brief 发送确认消息
     * @param cmd 确认的指令名称
     */
    void sendAck(const char* cmd) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "{\"type\":\"ack\",\"cmd\":\"%s\"}", cmd);
        broadcast(buffer);
    }

private:
    WebSocketsServer _server;       // WebSocket 服务器
    DACController* _dac;            // DAC 控制器引用
    SafetySystem* _safety;          // 安全系统引用
    PlayILDACallback _playILDACallback;  // ILDA 播放回调
    StopILDACallback _stopILDACallback;  // ILDA 停止回调
    bool _clientConnected;          // 客户端连接状态

    // ========================================================
    // 私有方法
    // ========================================================

    /**
     * @brief WebSocket 事件处理函数
     * @param num 客户端编号
     * @param type 事件类型
     * @param payload 数据负载
     * @param length 数据长度
     */
    void handleEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        switch (type) {
            case WStype_DISCONNECTED:
                Serial.printf("[WebSocket] 客户端 #%u 断开连接\n", num);
                _clientConnected = false;
                break;

            case WStype_CONNECTED:
                {
                    IPAddress ip = _server.remoteIP(num);
                    Serial.printf("[WebSocket] 客户端 #%u 已连接，IP: %d.%d.%d.%d\n",
                                  num, ip[0], ip[1], ip[2], ip[3]);
                    _clientConnected = true;

                    // 发送欢迎消息
                    char welcome[256];
                    snprintf(welcome, sizeof(welcome),
                             "{\"type\":\"welcome\","
                             "\"device\":\"GalvoEye-S3\","
                             "\"version\":\"1.0.0\","
                             "\"freeHeap\":%lu}",
                             ESP.getFreeHeap());
                    _server.sendTXT(num, welcome);
                }
                break;

            case WStype_TEXT:
                // 将 payload 转换为字符串
                {
                    char* text = (char*)payload;
                    // 确保字符串以 null 结尾
                    if (length < WS_RX_BUFFER_SIZE) {
                        char buffer[WS_RX_BUFFER_SIZE];
                        memcpy(buffer, text, length);
                        buffer[length] = '\0';
                        processCommand(num, buffer);
                    } else {
                        Serial.println("[WebSocket] 消息过长，已丢弃");
                        sendError("消息过长");
                    }
                }
                break;

            case WStype_ERROR:
                Serial.printf("[WebSocket] 客户端 #%u 发生错误\n", num);
                break;

            case WStype_FRAGMENT_TEXT_START:
            case WStype_FRAGMENT_BIN_START:
            case WStype_FRAGMENT:
            case WStype_FRAGMENT_FIN:
                // 分片消息暂不支持
                Serial.println("[WebSocket] 收到分片消息 (暂不支持)");
                break;

            default:
                break;
        }
    }

    /**
     * @brief 处理接收到的 JSON 指令
     * @param clientNum 客户端编号
     * @param jsonStr JSON 字符串
     */
    void processCommand(uint8_t clientNum, char* jsonStr) {
        // --- 简易 JSON 解析 ---
        // 注意: 这里使用手动字符串解析而非 ArduinoJson 库
        // 以减少依赖，同时保持代码可编译

        char cmd[32] = {0};

        // 提取 "cmd" 字段
        if (!extractJSONString(jsonStr, "cmd", cmd, sizeof(cmd))) {
            Serial.println("[WebSocket] 无法解析 cmd 字段");
            sendError("无效的指令格式");
            return;
        }

        Serial.printf("[WebSocket] 收到指令: %s\n", cmd);

        // --- 移动指令 ---
        if (strcmp(cmd, "moveTo") == 0) {
            if (!_dac) { sendError("DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendError("当前安全状态不允许操作");
                return;
            }

            int x = 0, y = 0;
            extractJSONInt(jsonStr, "x", &x);
            extractJSONInt(jsonStr, "y", &y);
            _dac->setTarget((uint16_t)x, (uint16_t)y);
            sendAck("moveTo");
        }
        // --- 绘制点 ---
        else if (strcmp(cmd, "drawPoint") == 0) {
            if (!_dac) { sendError("DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendError("当前安全状态不允许操作");
                return;
            }

            int x = 0, y = 0, r = 255, g = 255, b = 255;
            extractJSONInt(jsonStr, "x", &x);
            extractJSONInt(jsonStr, "y", &y);
            extractJSONInt(jsonStr, "r", &r);
            extractJSONInt(jsonStr, "g", &g);
            extractJSONInt(jsonStr, "b", &b);
            _dac->drawPoint((uint16_t)x, (uint16_t)y,
                            (uint8_t)r, (uint8_t)g, (uint8_t)b);
            sendAck("drawPoint");
        }
        // --- 绘制线 ---
        else if (strcmp(cmd, "drawLine") == 0) {
            if (!_dac) { sendError("DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendError("当前安全状态不允许操作");
                return;
            }

            int x1 = 0, y1 = 0, x2 = 0, y2 = 0, r = 255, g = 255, b = 255;
            extractJSONInt(jsonStr, "x1", &x1);
            extractJSONInt(jsonStr, "y1", &y1);
            extractJSONInt(jsonStr, "x2", &x2);
            extractJSONInt(jsonStr, "y2", &y2);
            extractJSONInt(jsonStr, "r", &r);
            extractJSONInt(jsonStr, "g", &g);
            extractJSONInt(jsonStr, "b", &b);
            _dac->drawLine((uint16_t)x1, (uint16_t)y1,
                           (uint16_t)x2, (uint16_t)y2,
                           (uint8_t)r, (uint8_t)g, (uint8_t)b);
            sendAck("drawLine");
        }
        // --- 绘制圆 ---
        else if (strcmp(cmd, "drawCircle") == 0) {
            if (!_dac) { sendError("DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendError("当前安全状态不允许操作");
                return;
            }

            int cx = 0, cy = 0, radius = 100, r = 255, g = 255, b = 255;
            extractJSONInt(jsonStr, "cx", &cx);
            extractJSONInt(jsonStr, "cy", &cy);
            extractJSONInt(jsonStr, "radius", &radius);
            extractJSONInt(jsonStr, "r", &r);
            extractJSONInt(jsonStr, "g", &g);
            extractJSONInt(jsonStr, "b", &b);
            _dac->drawCircle((uint16_t)cx, (uint16_t)cy,
                             (uint16_t)radius,
                             (uint8_t)r, (uint8_t)g, (uint8_t)b);
            sendAck("drawCircle");
        }
        // --- 绘制矩形 ---
        else if (strcmp(cmd, "drawRect") == 0) {
            if (!_dac) { sendError("DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendError("当前安全状态不允许操作");
                return;
            }

            int x = 0, y = 0, w = 100, h = 100, r = 255, g = 255, b = 255;
            extractJSONInt(jsonStr, "x", &x);
            extractJSONInt(jsonStr, "y", &y);
            extractJSONInt(jsonStr, "w", &w);
            extractJSONInt(jsonStr, "h", &h);
            extractJSONInt(jsonStr, "r", &r);
            extractJSONInt(jsonStr, "g", &g);
            extractJSONInt(jsonStr, "b", &b);
            _dac->drawRect((uint16_t)x, (uint16_t)y,
                           (uint16_t)w, (uint16_t)h,
                           (uint8_t)r, (uint8_t)g, (uint8_t)b);
            sendAck("drawRect");
        }
        // --- 绘制文字 ---
        else if (strcmp(cmd, "drawText") == 0) {
            if (!_dac) { sendError("DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendError("当前安全状态不允许操作");
                return;
            }

            char text[128] = {0};
            int x = 0, y = 0, scale = 3;
            extractJSONString(jsonStr, "text", text, sizeof(text));
            extractJSONInt(jsonStr, "x", &x);
            extractJSONInt(jsonStr, "y", &y);
            extractJSONInt(jsonStr, "scale", &scale);
            _dac->drawText(text, (uint16_t)x, (uint16_t)y, (uint8_t)scale);
            sendAck("drawText");
        }
        // --- 关闭激光 ---
        else if (strcmp(cmd, "laserOff") == 0) {
            if (_dac) _dac->laserOff();
            sendAck("laserOff");
        }
        // --- 开启激光 ---
        else if (strcmp(cmd, "laserOn") == 0) {
            if (!_dac) { sendError("DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendError("当前安全状态不允许开启激光");
                return;
            }

            int r = 255, g = 255, b = 255;
            extractJSONInt(jsonStr, "r", &r);
            extractJSONInt(jsonStr, "g", &g);
            extractJSONInt(jsonStr, "b", &b);
            _dac->setLaserColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
            sendAck("laserOn");
        }
        // --- 设置安全模式 ---
        else if (strcmp(cmd, "setSafety") == 0) {
            if (!_safety) { sendError("安全系统未初始化"); return; }

            char mode[32] = {0};
            extractJSONString(jsonStr, "mode", mode, sizeof(mode));

            if (strcmp(mode, "normal") == 0) {
                _safety->setManualOverride(false);
                _safety->setEnabled(true);
                _safety->clearEmergencyStop();
                sendAck("setSafety");
            } else if (strcmp(mode, "override") == 0) {
                _safety->setManualOverride(true);
                sendAck("setSafety");
            } else if (strcmp(mode, "off") == 0) {
                _safety->setEnabled(false);
                sendAck("setSafety");
            } else {
                sendError("未知的安全模式");
            }
        }
        // --- 请求状态 ---
        else if (strcmp(cmd, "getStatus") == 0) {
            sendStatus();
        }
        // --- 播放 ILDA 文件 ---
        else if (strcmp(cmd, "playILDA") == 0) {
            char filename[64] = {0};
            extractJSONString(jsonStr, "file", filename, sizeof(filename));

            if (strlen(filename) == 0) {
                sendError("文件名不能为空");
                return;
            }

            if (_playILDACallback) {
                _playILDACallback(filename);
                sendAck("playILDA");
            } else {
                sendError("ILDA 播放器未初始化");
            }
        }
        // --- 停止 ILDA 播放 ---
        else if (strcmp(cmd, "stopILDA") == 0) {
            if (_stopILDACallback) {
                _stopILDACallback();
                sendAck("stopILDA");
            } else {
                sendError("ILDA 播放器未初始化");
            }
        }
        // --- 回到中心 ---
        else if (strcmp(cmd, "home") == 0) {
            if (_dac) _dac->home();
            sendAck("home");
        }
        // --- 紧急停止 ---
        else if (strcmp(cmd, "emergencyStop") == 0) {
            if (_safety) {
                _safety->triggerEmergencyStop();
                if (_dac) _dac->laserOff();
                sendAck("emergencyStop");
            }
        }
        // --- 解除紧急停止 ---
        else if (strcmp(cmd, "clearEmergency") == 0) {
            if (_safety) {
                _safety->clearEmergencyStop();
                sendAck("clearEmergency");
            }
        }
        else {
            char errorMsg[64];
            snprintf(errorMsg, sizeof(errorMsg), "未知指令: %s", cmd);
            sendError(errorMsg);
        }
    }

    // ========================================================
    // 简易 JSON 解析工具函数
    // ========================================================

    /**
     * @brief 从 JSON 字符串中提取整数值
     * @param json JSON 字符串
     * @param key 键名
     * @param outValue 输出值
     * @return true 提取成功
     */
    static bool extractJSONInt(const char* json, const char* key, int* outValue) {
        if (!json || !key || !outValue) return false;

        // 构造搜索模式: "key":
        char pattern[32];
        snprintf(pattern, sizeof(pattern), "\"%s\":", key);

        const char* pos = strstr(json, pattern);
        if (!pos) {
            // 尝试带空格的格式: "key" :
            snprintf(pattern, sizeof(pattern), "\"%s\" :", key);
            pos = strstr(json, pattern);
            if (!pos) return false;
        }

        // 跳过键名和冒号
        pos += strlen(pattern);

        // 跳过空白
        while (*pos == ' ' || *pos == '\t') pos++;

        // 解析数字
        if (*pos == '-' || (*pos >= '0' && *pos <= '9')) {
            *outValue = atoi(pos);
            return true;
        }

        return false;
    }

    /**
     * @brief 从 JSON 字符串中提取字符串值
     * @param json JSON 字符串
     * @param key 键名
     * @param outValue 输出缓冲区
     * @param outSize 输出缓冲区大小
     * @return true 提取成功
     */
    static bool extractJSONString(const char* json, const char* key,
                                   char* outValue, size_t outSize) {
        if (!json || !key || !outValue || outSize == 0) return false;

        // 构造搜索模式: "key":"
        char pattern[64];
        snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);

        const char* pos = strstr(json, pattern);
        if (!pos) {
            // 尝试带空格的格式: "key" : "
            snprintf(pattern, sizeof(pattern), "\"%s\" : \"", key);
            pos = strstr(json, pattern);
            if (!pos) return false;
        }

        // 跳过键名、冒号和引号
        pos += strlen(pattern);

        // 查找结束引号
        const char* end = strchr(pos, '"');
        if (!end) return false;

        // 计算长度并复制
        size_t len = end - pos;
        if (len >= outSize) len = outSize - 1;
        memcpy(outValue, pos, len);
        outValue[len] = '\0';

        return true;
    }
};

#endif // WEBSOCKET_HANDLER_H
