/**
 * @file ilda_player.h
 * @brief ILDA 文件播放器 - SD 卡读取与帧率控制
 *
 * 本文件实现了 ILDA 标准格式文件的播放功能，包括:
 * - SD 卡初始化与文件管理
 * - ILDA 格式解析 (支持 Type 0, 1, 2, 4, 5)
 * - 帧率控制 (可调节)
 * - 播放/暂停/切换文件
 * - 文件列表扫描
 * - 播放状态管理
 *
 * ILDA 格式说明:
 * - Type 0: 3D 点数据，带索引颜色
 * - Type 1: 3D 点数据，带真彩色 (RGB)
 * - Type 2: 2D 点数据，带索引颜色
 * - Type 4: 2D 点数据，带真彩色 (RGB)
 * - Type 5: 颜色表 (调色板)
 *
 * @note 基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)
 */

#ifndef ILDA_PLAYER_H
#define ILDA_PLAYER_H

#include <Arduino.h>
#include <SD.h>
#include <FS.h>
#include "pin_defs.h"
#include "dac_controller.h"

// ============================================================
// ILDA 播放器参数
// ============================================================

// 文件缓冲区大小 (字节)
#define ILDA_READ_BUFFER_SIZE  4096

// 最大文件数
#define ILDA_MAX_FILES         50

// 文件名最大长度
#define ILDA_MAX_FILENAME_LEN  32

// 播放状态
enum ILDAPlayState {
    ILDA_IDLE = 0,          // 空闲状态
    ILDA_PLAYING = 1,       // 播放中
    ILDA_PAUSED = 2,        // 暂停
    ILDA_STOPPED = 3,       // 已停止
    ILDA_ERROR = 4          // 错误状态
};

/**
 * @brief 将播放状态转换为字符串
 */
static inline const char* ildaStateToString(ILDAPlayState state) {
    switch (state) {
        case ILDA_IDLE:    return "IDLE";
        case ILDA_PLAYING: return "PLAYING";
        case ILDA_PAUSED:  return "PAUSED";
        case ILDA_STOPPED: return "STOPPED";
        case ILDA_ERROR:   return "ERROR";
        default:           return "UNKNOWN";
    }
}

// ============================================================
// ILDA 播放器类
// ============================================================

class ILDAPlayer {
public:
    /**
     * @brief 构造函数
     */
    ILDAPlayer() :
        _currentState(ILDA_IDLE),
        _currentFileIndex(-1),
        _totalFiles(0),
        _fps(ILDA_DEFAULT_FPS),
        _currentFrame(0),
        _totalFrames(0),
        _currentPoint(0),
        _totalPoints(0),
        _formatCode(0),
        _loopPlayback(true),
        _sdInitialized(false),
        _file(nullptr),
        _dac(nullptr),
        _lastFrameTime(0),
        _pointsPerFrame(0),
        _lastError("")
    {
        memset(_fileList, 0, sizeof(_fileList));
    }

    /**
     * @brief 析构函数
     */
    ~ILDAPlayer() {
        stop();
    }

    /**
     * @brief 初始化 SD 卡
     * @return true 初始化成功
     */
    bool begin() {
        Serial.println("[ILDA] 正在初始化 SD 卡...");

        // 初始化 SD 卡 (SPI 模式)
        pinMode(PIN_SD_CS, OUTPUT);
        digitalWrite(PIN_SD_CS, HIGH);

        if (!SD.begin(PIN_SD_CS)) {
            Serial.println("[ILDA] SD 卡初始化失败!");
            Serial.println("[ILDA] 请检查: 1) SD 卡已插入 2) 引脚连接正确 3) 卡格式为 FAT32");
            _currentState = ILDA_ERROR;
            _lastError = "SD 卡初始化失败";
            return false;
        }

        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("[ILDA] SD 卡初始化成功，容量: %llu MB\n", cardSize);

        // 扫描 ILDA 文件
        scanFiles();

        _sdInitialized = true;
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
     * @brief 扫描 SD 卡中的 ILDA 文件
     * @return 找到的文件数量
     */
    int scanFiles() {
        if (!_sdInitialized) {
            Serial.println("[ILDA] SD 卡未初始化");
            return 0;
        }

        _totalFiles = 0;

        File root = SD.open("/");
        if (!root) {
            Serial.println("[ILDA] 无法打开 SD 卡根目录");
            return 0;
        }

        Serial.println("[ILDA] 正在扫描 ILDA 文件...");

        while (true) {
            File entry = root.openNextFile();
            if (!entry) break; // 没有更多文件

            if (!entry.isDirectory()) {
                const char* name = entry.name();
                size_t nameLen = strlen(name);

                // 检查文件扩展名 (.ild 或 .ilda)
                if ((nameLen > 4 && strcasecmp(name + nameLen - 4, ".ild") == 0) ||
                    (nameLen > 5 && strcasecmp(name + nameLen - 5, ".ilda") == 0)) {

                    if (_totalFiles < ILDA_MAX_FILES) {
                        strncpy(_fileList[_totalFiles], name, ILDA_MAX_FILENAME_LEN - 1);
                        _fileList[_totalFiles][ILDA_MAX_FILENAME_LEN - 1] = '\0';
                        _totalFiles++;
                        Serial.printf("[ILDA]   找到: %s\n", name);
                    }
                }
            }
            entry.close();
        }
        root.close();

        Serial.printf("[ILDA] 扫描完成，共找到 %d 个 ILDA 文件\n", _totalFiles);
        return _totalFiles;
    }

    /**
     * @brief 获取文件数量
     * @return 文件数量
     */
    int getFileCount() const {
        return _totalFiles;
    }

    /**
     * @brief 获取指定索引的文件名
     * @param index 文件索引
     * @return 文件名 (无文件时返回空字符串)
     */
    const char* getFileName(int index) const {
        if (index >= 0 && index < _totalFiles) {
            return _fileList[index];
        }
        return "";
    }

    /**
     * @brief 获取当前文件名
     * @return 当前播放的文件名
     */
    const char* getCurrentFileName() const {
        if (_currentFileIndex >= 0 && _currentFileIndex < _totalFiles) {
            return _fileList[_currentFileIndex];
        }
        return "";
    }

    /**
     * @brief 播放指定文件
     * @param filename 文件名 (空则播放当前文件)
     * @return true 开始播放成功
     */
    bool play(const char* filename = nullptr) {
        if (!_sdInitialized) {
            _lastError = "SD 卡未初始化";
            _currentState = ILDA_ERROR;
            return false;
        }

        if (!_dac) {
            _lastError = "DAC 未设置";
            _currentState = ILDA_ERROR;
            return false;
        }

        // 关闭之前打开的文件
        if (_file) {
            _file->close();
            delete _file;
            _file = nullptr;
        }

        // 确定要播放的文件
        if (filename && strlen(filename) > 0) {
            // 查找指定文件
            int foundIndex = -1;
            for (int i = 0; i < _totalFiles; i++) {
                if (strcmp(_fileList[i], filename) == 0) {
                    foundIndex = i;
                    break;
                }
            }

            if (foundIndex < 0) {
                Serial.printf("[ILDA] 文件未找到: %s\n", filename);
                _lastError = "文件未找到";
                _currentState = ILDA_ERROR;
                return false;
            }
            _currentFileIndex = foundIndex;
        } else {
            // 播放当前文件或第一个文件
            if (_currentFileIndex < 0) {
                if (_totalFiles > 0) {
                    _currentFileIndex = 0;
                } else {
                    Serial.println("[ILDA] 没有可播放的文件");
                    _lastError = "没有可播放的文件";
                    _currentState = ILDA_ERROR;
                    return false;
                }
            }
        }

        // 打开文件
        _file = new File(SD.open(_fileList[_currentFileIndex], FILE_READ));
        if (!_file || !_file->available()) {
            Serial.printf("[ILDA] 无法打开文件: %s\n", _fileList[_currentFileIndex]);
            _lastError = "无法打开文件";
            _currentState = ILDA_ERROR;
            if (_file) { delete _file; _file = nullptr; }
            return false;
        }

        // 解析文件头
        DACController::ILDAHeader header;
        uint8_t headerBuffer[sizeof(DACController::ILDAHeader)];
        if (_file->read(headerBuffer, sizeof(headerBuffer)) != sizeof(headerBuffer)) {
            Serial.println("[ILDA] 文件头读取失败");
            _lastError = "文件头读取失败";
            _currentState = ILDA_ERROR;
            _file->close();
            delete _file;
            _file = nullptr;
            return false;
        }

        if (!_dac->parseILDAHeader(headerBuffer, sizeof(headerBuffer), header)) {
            _lastError = "无效的 ILDA 文件";
            _currentState = ILDA_ERROR;
            _file->close();
            delete _file;
            _file = nullptr;
            return false;
        }

        _formatCode = header.formatCode;
        _totalPoints = header.totalPoints;
        _totalFrames = header.totalFrames;
        _currentFrame = 0;
        _currentPoint = 0;
        _lastFrameTime = millis();

        // 计算每帧点数
        _pointsPerFrame = _totalPoints;
        if (_totalFrames > 0) {
            _pointsPerFrame = _totalPoints / _totalFrames;
        }

        _currentState = ILDA_PLAYING;
        Serial.printf("[ILDA] 开始播放: %s (格式:%d, 点数:%d, 帧:%d)\n",
                      _fileList[_currentFileIndex],
                      _formatCode, _totalPoints, _totalFrames);

        return true;
    }

    /**
     * @brief 暂停播放
     */
    void pause() {
        if (_currentState == ILDA_PLAYING) {
            _currentState = ILDA_PAUSED;
            Serial.println("[ILDA] 播放已暂停");
        }
    }

    /**
     * @brief 恢复播放
     */
    void resume() {
        if (_currentState == ILDA_PAUSED) {
            _currentState = ILDA_PLAYING;
            _lastFrameTime = millis();
            Serial.println("[ILDA] 播放已恢复");
        }
    }

    /**
     * @brief 停止播放
     */
    void stop() {
        if (_file) {
            _file->close();
            delete _file;
            _file = nullptr;
        }

        if (_currentState == ILDA_PLAYING || _currentState == ILDA_PAUSED) {
            Serial.println("[ILDA] 播放已停止");
        }

        _currentState = ILDA_IDLE;
        _currentPoint = 0;
        _currentFrame = 0;

        // 关闭激光
        if (_dac) {
            _dac->laserOff();
            _dac->home();
        }
    }

    /**
     * @brief 播放下一个文件
     * @return true 成功切换
     */
    bool playNext() {
        if (_totalFiles == 0) return false;

        stop();
        _currentFileIndex = (_currentFileIndex + 1) % _totalFiles;
        return play();
    }

    /**
     * @brief 播放上一个文件
     * @return true 成功切换
     */
    bool playPrevious() {
        if (_totalFiles == 0) return false;

        stop();
        _currentFileIndex = (_currentFileIndex - 1 + _totalFiles) % _totalFiles;
        return play();
    }

    /**
     * @brief 设置帧率
     * @param fps 目标帧率 (10-60)
     */
    void setFPS(uint8_t fps) {
        _fps = constrain(fps, ILDA_MIN_FPS, ILDA_MAX_FPS);
        Serial.printf("[ILDA] 帧率设置为: %d FPS\n", _fps);
    }

    /**
     * @brief 获取当前帧率
     */
    uint8_t getFPS() const {
        return _fps;
    }

    /**
     * @brief 设置循环播放
     * @param loop true=循环, false=单次
     */
    void setLoop(bool loop) {
        _loopPlayback = loop;
    }

    /**
     * @brief 获取播放状态
     */
    ILDAPlayState getState() const {
        return _currentState;
    }

    /**
     * @brief 获取当前帧编号
     */
    uint16_t getCurrentFrame() const {
        return _currentFrame;
    }

    /**
     * @brief 获取总帧数
     */
    uint16_t getTotalFrames() const {
        return _totalFrames;
    }

    /**
     * @brief 获取最后错误信息
     */
    const char* getLastError() const {
        return _lastError;
    }

    /**
     * @brief 更新循环 (在主循环中调用)
     *
     * 本函数负责:
     * 1. 检查是否到达帧时间
     * 2. 读取并绘制当前帧的所有点
     * 3. 处理帧切换和循环
     */
    void update() {
        if (_currentState != ILDA_PLAYING) return;
        if (!_file || !_dac) return;

        // 帧率控制
        unsigned long now = millis();
        unsigned long frameInterval = 1000UL / _fps;
        if (now - _lastFrameTime < frameInterval) return;
        _lastFrameTime = now;

        // 读取并绘制当前帧的点
        bool frameComplete = renderFrame();

        if (frameComplete) {
            _currentFrame++;

            // 检查是否播放完毕
            if (_currentFrame >= _totalFrames) {
                if (_loopPlayback) {
                    // 循环播放: 重新打开文件
                    _currentFrame = 0;
                    _currentPoint = 0;
                    _file->seek(sizeof(DACController::ILDAHeader));
                } else {
                    // 单次播放: 停止
                    Serial.println("[ILDA] 播放完成");
                    stop();
                }
            }
        }
    }

    /**
     * @brief 获取播放状态 JSON 字符串
     * @param buffer 输出缓冲区
     * @param bufferSize 缓冲区大小
     */
    void getStatusJSON(char* buffer, size_t bufferSize) const {
        snprintf(buffer, bufferSize,
                 "{\"ilda\":{\"state\":\"%s\",\"stateCode\":%d,"
                 "\"file\":\"%s\",\"fileIndex\":%d,\"totalFiles\":%d,"
                 "\"frame\":%d,\"totalFrames\":%d,\"fps\":%d,"
                 "\"loop\":%s}}",
                 ildaStateToString(_currentState),
                 (int)_currentState,
                 getCurrentFileName(),
                 _currentFileIndex,
                 _totalFiles,
                 _currentFrame,
                 _totalFrames,
                 _fps,
                 _loopPlayback ? "true" : "false");
    }

private:
    // ========================================================
    // 私有成员变量
    // ========================================================
    ILDAPlayState _currentState;       // 当前播放状态
    int _currentFileIndex;             // 当前文件索引
    int _totalFiles;                   // 总文件数
    uint8_t _fps;                      // 目标帧率
    uint16_t _currentFrame;            // 当前帧编号
    uint16_t _totalFrames;             // 总帧数
    uint16_t _currentPoint;            // 当前点索引
    uint16_t _totalPoints;             // 总点数
    uint8_t _formatCode;               // ILDA 格式代码
    bool _loopPlayback;                // 循环播放标志
    bool _sdInitialized;               // SD 卡初始化标志
    File* _file;                       // 当前打开的文件
    DACController* _dac;               // DAC 控制器引用
    unsigned long _lastFrameTime;      // 上一帧时间
    uint16_t _pointsPerFrame;          // 每帧点数
    const char* _lastError;            // 最后错误信息
    char _fileList[ILDA_MAX_FILES][ILDA_MAX_FILENAME_LEN]; // 文件列表

    // ========================================================
    // 私有方法
    // ========================================================

    /**
     * @brief 渲染当前帧
     * @return true 帧渲染完成
     */
    bool renderFrame() {
        if (!_file || !_dac) return false;

        uint16_t pointsToRender = _pointsPerFrame;
        if (pointsToRender == 0) pointsToRender = _totalPoints;

        for (uint16_t i = 0; i < pointsToRender; i++) {
            if (!_file->available()) return true; // 文件结束

            // 根据格式代码读取点数据
            if (_formatCode == 0 || _formatCode == 1) {
                // 3D 格式 (Type 0: 索引颜色, Type 1: 真彩色)
                DACController::ILDAPoint3D point;
                if (_file->read((uint8_t*)&point, sizeof(point)) != sizeof(point)) {
                    return true; // 读取失败，帧结束
                }

                // 转换坐标
                uint16_t x = DACController::ildaToDAC(point.x);
                uint16_t y = DACController::ildaToDAC(point.y);

                // 检查是否为空白点
                if (DACController::isBlankedPoint(point.status)) {
                    _dac->laserOff();
                } else {
                    // 根据格式设置颜色
                    if (_formatCode == 1) {
                        // Type 1: 真彩色 (颜色信息在额外字节中)
                        // 注意: 标准 ILDA Type 1 在点数据后还有 2 字节颜色
                        uint8_t colorData[2];
                        if (_file->read(colorData, 2) == 2) {
                            // 简单的颜色映射
                            _dac->setLaserColor(255, 255, 255);
                        }
                    } else {
                        // Type 0: 使用索引颜色 (简化处理，使用白色)
                        _dac->setLaserColor(255, 255, 255);
                    }
                }

                // 移动到目标位置
                _dac->setTarget(x, y);
                _dac->update(0.9f);

                // 检查是否为帧最后一个点
                if (DACController::isLastPoint(point.status)) {
                    return true;
                }

            } else if (_formatCode == 2 || _formatCode == 4) {
                // 2D 格式 (Type 2: 索引颜色, Type 4: 真彩色)
                DACController::ILDAPoint2D point;
                if (_file->read((uint8_t*)&point, sizeof(point)) != sizeof(point)) {
                    return true;
                }

                uint16_t x = DACController::ildaToDAC(point.x);
                uint16_t y = DACController::ildaToDAC(point.y);

                if (DACController::isBlankedPoint(point.status)) {
                    _dac->laserOff();
                } else {
                    if (_formatCode == 4) {
                        uint8_t colorData[2];
                        if (_file->read(colorData, 2) == 2) {
                            _dac->setLaserColor(255, 255, 255);
                        }
                    } else {
                        _dac->setLaserColor(255, 255, 255);
                    }
                }

                _dac->setTarget(x, y);
                _dac->update(0.9f);

                if (DACController::isLastPoint(point.status)) {
                    return true;
                }

            } else if (_formatCode == 5) {
                // Type 5: 颜色表 (调色板)
                // 跳过颜色表数据
                DACController::ILDAColorTable color;
                if (_file->read((uint8_t*)&color, sizeof(color)) != sizeof(color)) {
                    return true;
                }
                // 颜色表数据不绘制，继续读取
            }
        }

        return true;
    }
};

#endif // ILDA_PLAYER_H
