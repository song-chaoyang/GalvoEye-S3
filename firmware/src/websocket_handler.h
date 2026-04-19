/**
 * @file websocket_handler.h
 * @brief WebSocket 通信处理器 - PC 端指令接收与状态上报
 *
 * 本文件实现了基于 WebSockets 库的通信处理器，负责:
 * - 接收 PC 端发送的目标坐标指令
 * - 接收绘图指令 (点、线、圆、矩形、文字)
 * - 接收安全模式切换指令 (需二次确认)
 * - 接收安全恢复确认指令
 * - 接收 ILDA 播放控制指令
 * - 发送设备状态信息回 PC 端
 * - 客户端连接/断开管理
 * - Token 认证机制
 *
 * 通信协议 (JSON 格式):
 * 接收指令格式:
 *   {"cmd":"auth","token":"GalvoEye2026"}       - 认证 (首次连接必须)
 *   {"cmd":"moveTo", "x":2048, "y":2048}         - 移动到坐标
 *   {"cmd":"drawPoint", "x":1000, "y":1000, "r":255, "g":0, "b":0} - 绘制点
 *   {"cmd":"drawLine", "x1":0, "y1":0, "x2":4095, "y2":4095, "r":255, "g":255, "b":255} - 绘制线
 *   {"cmd":"drawCircle", "cx":2048, "cy":2048, "radius":500, "r":0, "g":255, "b":0} - 绘制圆
 *   {"cmd":"drawRect", "x":500, "y":500, "w":3000, "h":3000, "r":0, "g":0, "b":255} - 绘制矩形
 *   {"cmd":"drawText", "text":"Hello", "x":100, "y":100, "scale":3} - 绘制文字
 *   {"cmd":"laserOff"}                              - 关闭激光
 *   {"cmd":"laserOn", "r":255, "g":255, "b":255}   - 开启激光
 *   {"cmd":"setSafety", "mode":"normal|override|off"} - 设置安全模式 (off 需二次确认)
 *   {"cmd":"confirmRecovery"}                       - 确认安全恢复
 *   {"cmd":"getStatus"}                             - 请求状态
 *   {"cmd":"playILDA", "file":"test.ild"}           - 播放 ILDA 文件
 *   {"cmd":"stopILDA"}                              - 停止 ILDA 播放
 *   {"cmd":"home"}                                  - 回到中心位置
 *   {"cmd":"emergencyStop"}                         - 紧急停止
 *   {"cmd":"clearEmergency"}                        - 解除紧急停止
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

// 最大客户端连接数 (用于认证状态跟踪)
#define WS_MAX_CLIENTS_TRACKED  WS_MAX_CLIENTS

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
        // 初始化所有客户端的认证状态为未认证
        for (int i = 0; i < WS_MAX_CLIENTS_TRACKED; i++) {
            _clientAuthenticated[i] = false;
            _safetyOffPending[i] = false;
        }
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
        Serial.printf("[WebSocket] 认证 Token: %s\n", WS_AUTH_TOKEN);
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
     * @brief 向指定客户端发送错误消息
     * @param num 客户端编号
     * @param errorMsg 错误描述
     */
    void sendErrorTo(uint8_t num, const char* errorMsg) {
        char buffer[WS_TX_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer),
                 "{\"type\":\"error\",\"message\":\"%s\"}", errorMsg);
        _server.sendTXT(num, buffer);
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

    /**
     * @brief 向指定客户端发送确认消息
     * @param num 客户端编号
     * @param cmd 确认的指令名称
     */
    void sendAckTo(uint8_t num, const char* cmd) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "{\"type\":\"ack\",\"cmd\":\"%s\"}", cmd);
        _server.sendTXT(num, buffer);
    }

private:
    WebSocketsServer _server;       // WebSocket 服务器
    DACController* _dac;            // DAC 控制器引用
    SafetySystem* _safety;          // 安全系统引用
    PlayILDACallback _playILDACallback;  // ILDA 播放回调
    StopILDACallback _stopILDACallback;  // ILDA 停止回调
    bool _clientConnected;          // 客户端连接状态

    // 客户端认证状态 (按客户端编号索引)
    bool _clientAuthenticated[WS_MAX_CLIENTS_TRACKED];
    // 安全系统禁用二次确认状态
    bool _safetyOffPending[WS_MAX_CLIENTS_TRACKED];

    // ========================================================
    // 私有方法
    // ========================================================

    /**
     * @brief 检查客户端是否已认证
     * @param num 客户端编号
     * @return true 已认证
     */
    bool isAuthenticated(uint8_t num) {
        if (num < WS_MAX_CLIENTS_TRACKED) {
            return _clientAuthenticated[num];
        }
        return false;
    }

    /**
     * @brief 设置客户端认证状态
     * @param num 客户端编号
     * @param auth 认证状态
     */
    void setAuthenticated(uint8_t num, bool auth) {
        if (num < WS_MAX_CLIENTS_TRACKED) {
            _clientAuthenticated[num] = auth;
        }
    }

    /**
     * @brief 检查安全禁用是否处于待确认状态
     * @param num 客户端编号
     * @return true 待确认
     */
    bool isSafetyOffPending(uint8_t num) {
        if (num < WS_MAX_CLIENTS_TRACKED) {
            return _safetyOffPending[num];
        }
        return false;
    }

    /**
     * @brief 设置安全禁用待确认状态
     * @param num 客户端编号
     * @param pending 待确认状态
     */
    void setSafetyOffPending(uint8_t num, bool pending) {
        if (num < WS_MAX_CLIENTS_TRACKED) {
            _safetyOffPending[num] = pending;
        }
    }

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
                // 清除该客户端的认证状态
                setAuthenticated(num, false);
                setSafetyOffPending(num, false);
                _clientConnected = false;
                break;

            case WStype_CONNECTED:
                {
                    IPAddress ip = _server.remoteIP(num);
                    Serial.printf("[WebSocket] 客户端 #%u 已连接，IP: %d.%d.%d.%d\n",
                                  num, ip[0], ip[1], ip[2], ip[3]);
                    _clientConnected = true;

                    // 新连接默认未认证
                    setAuthenticated(num, false);
                    setSafetyOffPending(num, false);

                    // 发送欢迎消息，提示需要认证
                    char welcome[256];
                    snprintf(welcome, sizeof(welcome),
                             "{\"type\":\"welcome\","
                             "\"device\":\"GalvoEye-S3\","
                             "\"version\":\"1.0.0\","
                             "\"freeHeap\":%lu,"
                             "\"authRequired\":true,"
                             "\"message\":\"请发送认证指令: {\\\"cmd\\\":\\\"auth\\\",\\\"token\\\":\\\"<token>\\\"}\"}",
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
                        sendErrorTo(num, "消息过长");
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
            sendErrorTo(clientNum, "无效的指令格式");
            return;
        }

        Serial.printf("[WebSocket] 客户端 #%u 指令: %s\n", clientNum, cmd);

        // --- 认证指令 (不需要预先认证) ---
        if (strcmp(cmd, "auth") == 0) {
            char token[64] = {0};
            extractJSONString(jsonStr, "token", token, sizeof(token));

            if (strcmp(token, WS_AUTH_TOKEN) == 0) {
                setAuthenticated(clientNum, true);
                Serial.printf("[WebSocket] 客户端 #%u 认证成功\n", clientNum);
                sendAckTo(clientNum, "auth");
            } else {
                Serial.printf("[WebSocket] 客户端 #%u 认证失败: Token 错误\n", clientNum);
                sendErrorTo(clientNum, "认证失败: Token 错误");
            }
            return;
        }

        // --- 以下指令需要先认证 ---
        if (!isAuthenticated(clientNum)) {
            Serial.printf("[WebSocket] 客户端 #%u 未认证，拒绝指令: %s\n", clientNum, cmd);
            sendErrorTo(clientNum, "未认证，请先发送认证指令");
            return;
        }

        // --- 移动指令 ---
        if (strcmp(cmd, "moveTo") == 0) {
            if (!_dac) { sendErrorTo(clientNum, "DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendErrorTo(clientNum, "当前安全状态不允许操作");
                return;
            }

            int x = 0, y = 0;
            extractJSONInt(jsonStr, "x", &x);
            extractJSONInt(jsonStr, "y", &y);
            _dac->setTarget((uint16_t)x, (uint16_t)y);
            sendAckTo(clientNum, "moveTo");
        }
        // --- 绘制点 ---
        else if (strcmp(cmd, "drawPoint") == 0) {
            if (!_dac) { sendErrorTo(clientNum, "DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendErrorTo(clientNum, "当前安全状态不允许操作");
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
            sendAckTo(clientNum, "drawPoint");
        }
        // --- 绘制线 ---
        else if (strcmp(cmd, "drawLine") == 0) {
            if (!_dac) { sendErrorTo(clientNum, "DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendErrorTo(clientNum, "当前安全状态不允许操作");
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
            sendAckTo(clientNum, "drawLine");
        }
        // --- 绘制圆 ---
        else if (strcmp(cmd, "drawCircle") == 0) {
            if (!_dac) { sendErrorTo(clientNum, "DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendErrorTo(clientNum, "当前安全状态不允许操作");
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
            sendAckTo(clientNum, "drawCircle");
        }
        // --- 绘制矩形 ---
        else if (strcmp(cmd, "drawRect") == 0) {
            if (!_dac) { sendErrorTo(clientNum, "DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendErrorTo(clientNum, "当前安全状态不允许操作");
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
            sendAckTo(clientNum, "drawRect");
        }
        // --- 绘制文字 ---
        else if (strcmp(cmd, "drawText") == 0) {
            if (!_dac) { sendErrorTo(clientNum, "DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendErrorTo(clientNum, "当前安全状态不允许操作");
                return;
            }

            char text[128] = {0};
            int x = 0, y = 0, scale = 3;
            extractJSONString(jsonStr, "text", text, sizeof(text));
            extractJSONInt(jsonStr, "x", &x);
            extractJSONInt(jsonStr, "y", &y);
            extractJSONInt(jsonStr, "scale", &scale);
            _dac->drawText(text, (uint16_t)x, (uint16_t)y, (uint8_t)scale);
            sendAckTo(clientNum, "drawText");
        }
        // --- 关闭激光 ---
        else if (strcmp(cmd, "laserOff") == 0) {
            if (_dac) _dac->laserOff();
            sendAckTo(clientNum, "laserOff");
        }
        // --- 开启激光 ---
        else if (strcmp(cmd, "laserOn") == 0) {
            if (!_dac) { sendErrorTo(clientNum, "DAC 未初始化"); return; }
            if (!_safety || !_safety->isSafeToOperate()) {
                sendErrorTo(clientNum, "当前安全状态不允许开启激光");
                return;
            }

            int r = 255, g = 255, b = 255;
            extractJSONInt(jsonStr, "r", &r);
            extractJSONInt(jsonStr, "g", &g);
            extractJSONInt(jsonStr, "b", &b);
            _dac->setLaserColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
            sendAckTo(clientNum, "laserOn");
        }
        // --- 设置安全模式 (off 需要二次确认) ---
        else if (strcmp(cmd, "setSafety") == 0) {
            if (!_safety) { sendErrorTo(clientNum, "安全系统未初始化"); return; }

            char mode[32] = {0};
            extractJSONString(jsonStr, "mode", mode, sizeof(mode));

            if (strcmp(mode, "normal") == 0) {
                _safety->setManualOverride(false);
                _safety->setEnabled(true);
                _safety->clearEmergencyStop();
                setSafetyOffPending(clientNum, false);
                sendAckTo(clientNum, "setSafety");
            } else if (strcmp(mode, "override") == 0) {
                _safety->setManualOverride(true);
                sendAckTo(clientNum, "setSafety");
            } else if (strcmp(mode, "off") == 0) {
                // 安全系统禁用需要二次确认
                if (!isSafetyOffPending(clientNum)) {
                    // 第一次发送: 进入待确认状态
                    setSafetyOffPending(clientNum, true);
                    Serial.printf("[WebSocket] 客户端 #%u 请求禁用安全系统，等待二次确认\n", clientNum);
                    sendErrorTo(clientNum, "安全系统禁用需要二次确认，请再次发送 setSafety off 确认");
                } else {
                    // 第二次发送: 执行禁用
                    setSafetyOffPending(clientNum, false);
                    _safety->setEnabled(false);
                    Serial.printf("[WebSocket] 客户端 #%u 已二次确认，安全系统已禁用\n", clientNum);
                    sendAckTo(clientNum, "setSafety");
                }
            } else {
                sendErrorTo(clientNum, "未知的安全模式");
            }
        }
        // --- 确认安全恢复 ---
        else if (strcmp(cmd, "confirmRecovery") == 0) {
            if (!_safety) { sendErrorTo(clientNum, "安全系统未初始化"); return; }

            if (_safety->isRecoveryPending()) {
                bool success = _safety->confirmRecovery();
                if (success) {
                    sendAckTo(clientNum, "confirmRecovery");
                } else {
                    sendErrorTo(clientNum, "恢复确认失败，当前环境仍不安全");
                }
            } else {
                sendErrorTo(clientNum, "当前不在恢复待定状态");
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
                sendErrorTo(clientNum, "文件名不能为空");
                return;
            }

            if (_playILDACallback) {
                _playILDACallback(filename);
                sendAckTo(clientNum, "playILDA");
            } else {
                sendErrorTo(clientNum, "ILDA 播放器未初始化");
            }
        }
        // --- 停止 ILDA 播放 ---
        else if (strcmp(cmd, "stopILDA") == 0) {
            if (_stopILDACallback) {
                _stopILDACallback();
                sendAckTo(clientNum, "stopILDA");
            } else {
                sendErrorTo(clientNum, "ILDA 播放器未初始化");
            }
        }
        // --- 回到中心 ---
        else if (strcmp(cmd, "home") == 0) {
            if (_dac) _dac->home();
            sendAckTo(clientNum, "home");
        }
        // --- 紧急停止 ---
        else if (strcmp(cmd, "emergencyStop") == 0) {
            if (_safety) {
                _safety->triggerEmergencyStop();
                if (_dac) _dac->laserOff();
                sendAckTo(clientNum, "emergencyStop");
            }
        }
        // --- 解除紧急停止 ---
        else if (strcmp(cmd, "clearEmergency") == 0) {
            if (_safety) {
                _safety->clearEmergencyStop();
                sendAckTo(clientNum, "clearEmergency");
            }
        }
        else {
            char errorMsg[64];
            snprintf(errorMsg, sizeof(errorMsg), "未知指令: %s", cmd);
            sendErrorTo(clientNum, errorMsg);
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
