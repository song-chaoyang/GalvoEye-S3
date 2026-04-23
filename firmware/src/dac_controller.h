/**
 * @file dac_controller.h
 * @brief MCP4822 DAC 控制器 - 振镜驱动与矢量图形绘制
 *
 * 本文件实现了基于 MCP4822 双通道 12 位 DAC 的振镜控制系统。
 * MCP4822 通过 SPI 接口通信，通道 A 控制 X 轴振镜，
 * 通道 B 控制 Y 轴振镜。
 *
 * 功能包括:
 * - SPI 总线初始化与 DAC 通信
 * - 双通道模拟输出 (X/Y 振镜)
 * - 平滑插值 (lerp) 避免振镜跳变
 * - 矢量图形绘制: 点、线、圆、矩形
 * - ILDA 文件格式解析与播放
 *
 * @note 基于 bbLaser 项目衍生 (CC-BY-NC-SA 3.0)
 */

#ifndef DAC_CONTROLLER_H
#define DAC_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include "pin_defs.h"

// ============================================================
// MCP4822 寄存器与命令定义
// ============================================================

// MCP4822 命令位定义
// bit 15: /CS (0 = 选中)
// bit 14-13: 未使用
// bit 12: /GA (0 = 增益 2x, 1 = 增益 1x)
// bit 11: /SHDN (0 = 关断, 1 = 正常工作)
// bit 10-0: 数据 (11位有效，左对齐到12位)

#define MCP4822_CH_A          0x0000  // 通道 A 选择位
#define MCP4822_CH_B          0x8000  // 通道 B 选择位
#define MCP4822_GAIN_1X       0x2000  // 增益 1x (输出 0-Vref)
#define MCP4822_GAIN_2X       0x0000  // 增益 2x (输出 0-2*Vref)
#define MCP4822_ACTIVE        0x1000  // 正常工作模式
#define MCP4822_SHUTDOWN      0x0000  // 关断模式

// ============================================================
// DAC 控制器类
// ============================================================

class DACController {
public:
    /**
     * @brief 构造函数
     */
    DACController() :
        _currentX(DAC_MID_VALUE),
        _currentY(DAC_MID_VALUE),
        _targetX(DAC_MID_VALUE),
        _targetY(DAC_MID_VALUE),
        _laserR(0),
        _laserG(0),
        _laserB(0),
        _initialized(false),
        _spiSettings(SPISettings(10000000, MSBFIRST, SPI_MODE0))
    {
    }

    /**
     * @brief 初始化 DAC 和 SPI 总线
     * @return true 初始化成功, false 初始化失败
     */
    bool begin() {
        // 配置 DAC CS 引脚
        pinMode(PIN_DAC_SPI_CS, OUTPUT);
        digitalWrite(PIN_DAC_SPI_CS, HIGH);

        // 配置激光 PWM 引脚
        pinMode(PIN_LASER_R, OUTPUT);
        pinMode(PIN_LASER_G, OUTPUT);
        pinMode(PIN_LASER_B, OUTPUT);

        // 初始化激光 PWM (默认关闭)
        ledcSetup(0, LASER_PWM_FREQ, LASER_PWM_RESOLUTION); // 红色通道
        ledcSetup(1, LASER_PWM_FREQ, LASER_PWM_RESOLUTION); // 绿色通道
        ledcSetup(2, LASER_PWM_FREQ, LASER_PWM_RESOLUTION); // 蓝色通道
        ledcAttachPin(PIN_LASER_R, 0);
        ledcAttachPin(PIN_LASER_G, 1);
        ledcAttachPin(PIN_LASER_B, 2);

        // 关闭激光
        setLaserColor(0, 0, 0);

        // 初始化 SPI 总线
        SPI.begin(PIN_DAC_SPI_SCK, PIN_DAC_SPI_MISO, PIN_DAC_SPI_MOSI, PIN_DAC_SPI_CS);

        // 将振镜移到中心位置
        setTarget(DAC_MID_VALUE, DAC_MID_VALUE);
        update();

        _initialized = true;
        Serial.println("[DAC] MCP4822 初始化完成");
        return true;
    }

    /**
     * @brief 设置目标位置 (直接设置，不等待移动完成)
     * @param x X 轴目标值 (0 - DAC_MAX_VALUE)
     * @param y Y 轴目标值 (0 - DAC_MAX_VALUE)
     */
    void setTarget(uint16_t x, uint16_t y) {
        _targetX = constrain(x, 0, DAC_MAX_VALUE);
        _targetY = constrain(y, 0, DAC_MAX_VALUE);
    }

    /**
     * @brief 设置目标位置 (归一化坐标 -1.0 到 1.0)
     * @param nx X 轴归一化坐标
     * @param ny Y 轴归一化坐标
     */
    void setTargetNormalized(float nx, float ny) {
        // 将 -1.0 ~ 1.0 映射到 0 ~ DAC_MAX_VALUE
        uint16_t x = (uint16_t)((nx + 1.0f) * 0.5f * DAC_MAX_VALUE);
        uint16_t y = (uint16_t)((ny + 1.0f) * 0.5f * DAC_MAX_VALUE);
        setTarget(x, y);
    }

    /**
     * @brief 立即移动到目标位置 (无插值)
     */
    void moveImmediate() {
        _currentX = _targetX;
        _currentY = _targetY;
        writeDAC(_currentX, _currentY);
    }

    /**
     * @brief 更新当前位置 (带平滑插值)
     * @param speed 插值速度 (0-1, 1=立即到位)
     * @return true 已到达目标位置, false 仍在移动中
     */
    bool update(float speed = 0.3f) {
        if (!_initialized) return true;

        // 线性插值 (lerp)
        _currentX = lerp(_currentX, _targetX, speed);
        _currentY = lerp(_currentY, _targetY, speed);

        // 写入 DAC
        writeDAC((uint16_t)_currentX, (uint16_t)_currentY);

        // 判断是否已到达目标
        bool xDone = abs((int)_currentX - (int)_targetX) <= GALVO_MIN_STEP;
        bool yDone = abs((int)_currentY - (int)_targetY) <= GALVO_MIN_STEP;

        return xDone && yDone;
    }

    /**
     * @brief 获取当前 X 位置
     * @return 当前 X 轴 DAC 值
     */
    uint16_t getCurrentX() const { return (uint16_t)_currentX; }

    /**
     * @brief 获取当前 Y 位置
     * @return 当前 Y 轴 DAC 值
     */
    uint16_t getCurrentY() const { return (uint16_t)_currentY; }

    /**
     * @brief 获取目标 X 位置
     * @return 目标 X 轴 DAC 值
     */
    uint16_t getTargetX() const { return _targetX; }

    /**
     * @brief 获取目标 Y 位置
     * @return 目标 Y 轴 DAC 值
     */
    uint16_t getTargetY() const { return _targetY; }

    // ========================================================
    // 激光控制
    // ========================================================

    /**
     * @brief 设置激光颜色 (RGB)
     * @param r 红色亮度 (0-255)
     * @param g 绿色亮度 (0-255)
     * @param b 蓝色亮度 (0-255)
     */
    void setLaserColor(uint8_t r, uint8_t g, uint8_t b) {
        _laserR = constrain(r, 0, 255);
        _laserG = constrain(g, 0, 255);
        _laserB = constrain(b, 0, 255);
        ledcWrite(0, _laserR);
        ledcWrite(1, _laserG);
        ledcWrite(2, _laserB);
    }

    /**
     * @brief 关闭激光
     */
    void laserOff() {
        setLaserColor(0, 0, 0);
    }

    /**
     * @brief 设置激光亮度 (白色)
     * @param brightness 亮度 (0-255)
     */
    void setLaserBrightness(uint8_t brightness) {
        setLaserColor(brightness, brightness, brightness);
    }

    /**
     * @brief 获取当前激光颜色
     * @param r 输出红色值
     * @param g 输出绿色值
     * @param b 输出蓝色值
     */
    void getLaserColor(uint8_t &r, uint8_t &g, uint8_t &b) const {
        r = _laserR;
        g = _laserG;
        b = _laserB;
    }

    // ========================================================
    // 矢量图形绘制函数
    // ========================================================

    /**
     * @brief 绘制单个点 (激光开启，移动到指定位置)
     * @param x X 坐标 (0 - DAC_MAX_VALUE)
     * @param y Y 坐标 (0 - DAC_MAX_VALUE)
     * @param r 红色亮度
     * @param g 绿色亮度
     * @param b 蓝色亮度
     * @param dwellMs 停留时间 (毫秒)
     */
    void drawPoint(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b, uint16_t dwellMs = 1) {
        setTarget(x, y);
        // 快速移动到目标
        while (!update(0.8f)) {
            delayMicroseconds(10);
        }
        // 开启激光并停留
        setLaserColor(r, g, b);
        delay(dwellMs);
    }

    /**
     * @brief 绘制一条线段 (从当前位置到目标位置)
     * @param x1 起点X
     * @param y1 起点Y
     * @param x2 终点X
     * @param y2 终点Y
     * @param r 红色亮度
     * @param g 绿色亮度
     * @param b 蓝色亮度
     * @param stepSize 步进大小 (越小越平滑)
     */
    void drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                  uint8_t r, uint8_t g, uint8_t b, uint16_t stepSize = 10) {
        // 计算线段长度
        int32_t dx = (int32_t)x2 - (int32_t)x1;
        int32_t dy = (int32_t)y2 - (int32_t)y1;
        float length = sqrt((float)(dx * dx + dy * dy));

        if (length < 1.0f) {
            drawPoint(x1, y1, r, g, b);
            return;
        }

        // 计算步数
        uint16_t steps = (uint16_t)(length / stepSize);
        if (steps < 1) steps = 1;

        // 逐点绘制
        for (uint16_t i = 0; i <= steps; i++) {
            float t = (float)i / (float)steps;
            uint16_t px = (uint16_t)((float)x1 + (float)dx * t);
            uint16_t py = (uint16_t)((float)y1 + (float)dy * t);
            setTarget(px, py);
            update(0.9f);
            setLaserColor(r, g, b);
            delayMicroseconds(20);
        }
    }

    /**
     * @brief 绘制圆形
     * @param cx 圆心X
     * @param cy 圆心Y
     * @param radius 半径 (DAC 单位)
     * @param r 红色亮度
     * @param g 绿色亮度
     * @param b 蓝色亮度
     * @param segments 线段数量 (越多越圆滑)
     */
    void drawCircle(uint16_t cx, uint16_t cy, uint16_t radius,
                    uint8_t r, uint8_t g, uint8_t b, uint16_t segments = 64) {
        if (radius < 1) return;

        float angleStep = 2.0f * PI / segments;

        for (uint16_t i = 0; i <= segments; i++) {
            float angle = angleStep * i;
            uint16_t px = cx + (uint16_t)((float)radius * cos(angle));
            uint16_t py = cy + (uint16_t)((float)radius * sin(angle));
            setTarget(px, py);
            update(0.8f);
            setLaserColor(r, g, b);
            delayMicroseconds(20);
        }
    }

    /**
     * @brief 绘制矩形
     * @param x 左上角X
     * @param y 左上角Y
     * @param w 宽度
     * @param h 高度
     * @param r 红色亮度
     * @param g 绿色亮度
     * @param b 蓝色亮度
     */
    void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                  uint8_t r, uint8_t g, uint8_t b) {
        // 上边
        drawLine(x, y, x + w, y, r, g, b);
        // 右边
        drawLine(x + w, y, x + w, y + h, r, g, b);
        // 下边
        drawLine(x + w, y + h, x, y + h, r, g, b);
        // 左边
        drawLine(x, y + h, x, y, r, g, b);
    }

    /**
     * @brief 绘制填充矩形
     * @param x 左上角X
     * @param y 左上角Y
     * @param w 宽度
     * @param h 高度
     * @param r 红色亮度
     * @param g 绿色亮度
     * @param b 蓝色亮度
     * @param lineSpacing 扫描线间距
     */
    void drawFilledRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                        uint8_t r, uint8_t g, uint8_t b, uint16_t lineSpacing = 20) {
        for (uint16_t row = y; row <= y + h; row += lineSpacing) {
            drawLine(x, row, x + w, row, r, g, b, 10);
        }
    }

    /**
     * @brief 绘制文字 (简单 5x7 点阵)
     * @param text 文字内容
     * @param startX 起始X坐标
     * @param startY 起始Y坐标
     * @param scale 缩放比例
     * @param r 红色亮度
     * @param g 绿色亮度
     * @param b 蓝色亮度
     */
    void drawText(const char* text, uint16_t startX, uint16_t startY,
                  uint8_t scale = 3, uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) {
        uint16_t cursorX = startX;
        uint16_t cursorY = startY;

        while (*text) {
            char c = *text++;
            if (c == '\n') {
                cursorX = startX;
                cursorY += 7 * scale + scale;
                continue;
            }
            if (c < 32 || c > 126) continue; // 只支持可打印 ASCII

            // 获取字符点阵数据
            const uint8_t* charData = getCharBitmap(c);

            // 绘制字符的每一列
            for (uint8_t col = 0; col < 5; col++) {
                uint8_t colData = charData[col];
                for (uint8_t row = 0; row < 7; row++) {
                    if (colData & (1 << row)) {
                        // 绘制缩放后的像素点
                        for (uint8_t sy = 0; sy < scale; sy++) {
                            for (uint8_t sx = 0; sx < scale; sx++) {
                                uint16_t px = cursorX + col * scale + sx;
                                uint16_t py = cursorY + row * scale + sy;
                                drawPoint(px, py, r, g, b, 0);
                            }
                        }
                    }
                }
            }
            cursorX += 6 * scale; // 字符宽度 + 间距
        }
    }

    /**
     * @brief 将振镜移回中心位置并关闭激光
     */
    void home() {
        laserOff();
        setTarget(DAC_MID_VALUE, DAC_MID_VALUE);
        while (!update(0.2f)) {
            delayMicroseconds(100);
        }
    }

    // ========================================================
    // ILDA 文件解析
    // ========================================================

    // ILDA 格式使用大端序，ESP32-S3 是小端序，需要转换
    static inline uint16_t readBE16(const uint8_t* p) {
        return (uint16_t(p[0]) << 8) | p[1];
    }
    static inline int16_t readBE16Signed(const uint8_t* p) {
        return (int16_t)((uint16_t(p[0]) << 8) | p[1]);
    }

    /**
     * @brief ILDA 文件头结构
     */
    struct ILDAHeader {
        char    ildaStr[4];      // "ILDA"
        uint8_t formatCode;      // 格式代码 (0,1,2,4,5)
        char    projectName[8];  // 项目名称 (未使用)
        char    companyName[8];  // 公司名称 (未使用)
        uint16_t totalPoints;    // 总点数
        uint16_t frameNumber;    // 帧编号
        uint16_t totalFrames;    // 总帧数
        uint8_t  scannerHead;    // 扫描头编号
        uint8_t  reserved;       // 保留
    } __attribute__((packed));

    /**
     * @brief ILDA 数据点结构 (格式 0/1: 3D)
     */
    struct ILDAPoint3D {
        int16_t x;       // X 坐标 (-32768 ~ 32767)
        int16_t y;       // Y 坐标 (-32768 ~ 32767)
        int16_t z;       // Z 坐标 (-32768 ~ 32767)
        uint8_t status;  // 状态位 (bit6=blanking, bit7=last point)
        uint8_t colorIndex; // 颜色索引
    } __attribute__((packed));

    /**
     * @brief ILDA 数据点结构 (格式 2: 2D)
     */
    struct ILDAPoint2D {
        int16_t x;       // X 坐标 (-32768 ~ 32767)
        int16_t y;       // Y 坐标 (-32768 ~ 32767)
        uint8_t status;  // 状态位
        uint8_t colorIndex; // 颜色索引
    } __attribute__((packed));

    /**
     * @brief ILDA 颜色表结构
     */
    struct ILDAColorTable {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } __attribute__((packed));

    /**
     * @brief 解析 ILDA 文件头
     * @param data 文件数据指针
     * @param dataSize 数据大小
     * @param header 输出头信息
     * @return true 解析成功
     */
    bool parseILDAHeader(const uint8_t* data, size_t dataSize, ILDAHeader& header) {
        if (dataSize < sizeof(ILDAHeader)) {
            Serial.println("[ILDA] 文件数据不足，无法解析文件头");
            return false;
        }

        // 逐字段读取，处理大端序转换
        memcpy(header.ildaStr, data, 4);            // "ILDA" 标识 (ASCII，无需转换)
        header.formatCode = data[4];                 // 格式代码 (单字节，无需转换)
        memcpy(header.projectName, data + 5, 8);     // 项目名称 (ASCII，无需转换)
        memcpy(header.companyName, data + 13, 8);    // 公司名称 (ASCII，无需转换)
        header.totalPoints = readBE16(data + 21);    // 总点数 (大端序 uint16_t)
        header.frameNumber = readBE16(data + 23);    // 帧编号 (大端序 uint16_t)
        header.totalFrames = readBE16(data + 25);    // 总帧数 (大端序 uint16_t)
        header.scannerHead = data[27];               // 扫描头编号 (单字节，无需转换)
        header.reserved = data[28];                  // 保留 (单字节，无需转换)

        // 验证 ILDA 标识
        if (memcmp(header.ildaStr, "ILDA", 4) != 0) {
            Serial.println("[ILDA] 无效的 ILDA 文件标识");
            return false;
        }

        // 验证格式代码
        if (header.formatCode != 0 && header.formatCode != 1 &&
            header.formatCode != 2 && header.formatCode != 4 &&
            header.formatCode != 5) {
            Serial.printf("[ILDA] 不支持的格式代码: %d\n", header.formatCode);
            return false;
        }

        Serial.printf("[ILDA] 格式: %d, 点数: %d, 帧: %d/%d\n",
                      header.formatCode, header.totalPoints,
                      header.frameNumber, header.totalFrames);
        return true;
    }

    /**
     * @brief 将 ILDA 坐标转换为 DAC 值
     * @param ildaCoord ILDA 坐标 (-32768 ~ 32767)
     * @return DAC 值 (0 ~ DAC_MAX_VALUE)
     */
    static uint16_t ildaToDAC(int16_t ildaCoord) {
        // ILDA 坐标范围: -32768 ~ 32767
        // 映射到 DAC 范围: 0 ~ 4095
        int32_t val = (int32_t)ildaCoord + 32768L;
        val = val * DAC_MAX_VALUE / 65535L;
        return (uint16_t)constrain(val, 0, DAC_MAX_VALUE);
    }

    /**
     * @brief 检查 ILDA 点是否为空白点 (激光关闭)
     * @param status ILDA 状态字节
     * @return true 为空白点
     */
    static bool isBlankedPoint(uint8_t status) {
        return (status & 0x40) != 0; // bit6 = blanking
    }

    /**
     * @brief 检查是否为帧的最后一个点
     * @param status ILDA 状态字节
     * @return true 为最后一个点
     */
    static bool isLastPoint(uint8_t status) {
        return (status & 0x80) != 0; // bit7 = last point
    }

private:
    // ========================================================
    // 私有成员变量
    // ========================================================
    float _currentX;          // 当前 X 位置 (浮点，用于平滑插值)
    float _currentY;          // 当前 Y 位置
    uint16_t _targetX;        // 目标 X 位置
    uint16_t _targetY;        // 目标 Y 位置
    uint8_t _laserR;          // 当前红色亮度
    uint8_t _laserG;          // 当前绿色亮度
    uint8_t _laserB;          // 当前蓝色亮度
    bool _initialized;        // 初始化标志
    SPISettings _spiSettings; // SPI 通信参数

    // ========================================================
    // 私有方法
    // ========================================================

    /**
     * @brief 线性插值函数
     * @param a 起始值
     * @param b 目标值
     * @param t 插值因子 (0.0 - 1.0)
     * @return 插值结果
     */
    static float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }

    /**
     * @brief 向 MCP4822 写入双通道数据
     * @param xVal X 轴 DAC 值 (通道 A)
     * @param yVal Y 轴 DAC 值 (通道 B)
     */
    void writeDAC(uint16_t xVal, uint16_t yVal) {
        // 限制范围
        xVal = constrain(xVal, 0, DAC_MAX_VALUE);
        yVal = constrain(yVal, 0, DAC_MAX_VALUE);

        // MCP4822 使用 12 位数据，左对齐到 16 位
        // 通道 A: X 轴
        uint16_t cmdA = MCP4822_CH_A | MCP4822_GAIN_2X | MCP4822_ACTIVE | (xVal & 0x0FFF);
        // 通道 B: Y 轴
        uint16_t cmdB = MCP4822_CH_B | MCP4822_GAIN_2X | MCP4822_ACTIVE | (yVal & 0x0FFF);

        SPI.beginTransaction(_spiSettings);
        digitalWrite(PIN_DAC_SPI_CS, LOW);

        // 先写通道 A (X 轴)
        SPI.transfer16(cmdA);

        // 再写通道 B (Y 轴)
        SPI.transfer16(cmdB);

        digitalWrite(PIN_DAC_SPI_CS, HIGH);
        SPI.endTransaction();
    }

    /**
     * @brief 获取 ASCII 字符的 5x7 点阵数据
     * @param c ASCII 字符 (32-126)
     * @return 5 字节的点阵数据 (每字节一列，低位在上)
     */
    static const uint8_t* getCharBitmap(char c) {
        // 5x7 ASCII 点阵字体表 (从空格开始)
        // 每个字符 5 字节，每字节代表一列 (bit0=最上面的点)
        static const uint8_t font5x7[][5] = {
            {0x00,0x00,0x00,0x00,0x00}, // 空格 (32)
            {0x00,0x00,0x5F,0x00,0x00}, // !
            {0x00,0x07,0x00,0x07,0x00}, // "
            {0x14,0x7F,0x14,0x7F,0x14}, // #
            {0x24,0x2A,0x7F,0x2A,0x12}, // $
            {0x23,0x13,0x08,0x64,0x62}, // %
            {0x36,0x49,0x55,0x22,0x50}, // &
            {0x00,0x05,0x03,0x00,0x00}, // '
            {0x00,0x1C,0x22,0x41,0x00}, // (
            {0x00,0x41,0x22,0x1C,0x00}, // )
            {0x14,0x08,0x3E,0x08,0x14}, // *
            {0x08,0x08,0x3E,0x08,0x08}, // +
            {0x00,0x50,0x30,0x00,0x00}, // ,
            {0x08,0x08,0x08,0x08,0x08}, // -
            {0x00,0x60,0x60,0x00,0x00}, // .
            {0x20,0x10,0x08,0x04,0x02}, // /
            {0x3E,0x51,0x49,0x45,0x3E}, // 0
            {0x00,0x42,0x7F,0x40,0x00}, // 1
            {0x42,0x61,0x51,0x49,0x46}, // 2
            {0x21,0x41,0x45,0x4B,0x31}, // 3
            {0x18,0x14,0x12,0x7F,0x10}, // 4
            {0x27,0x45,0x45,0x45,0x39}, // 5
            {0x3C,0x4A,0x49,0x49,0x30}, // 6
            {0x01,0x71,0x09,0x05,0x03}, // 7
            {0x36,0x49,0x49,0x49,0x36}, // 8
            {0x06,0x49,0x49,0x29,0x1E}, // 9
            {0x00,0x36,0x36,0x00,0x00}, // :
            {0x00,0x56,0x36,0x00,0x00}, // ;
            {0x08,0x14,0x22,0x41,0x00}, // <
            {0x14,0x14,0x14,0x14,0x14}, // =
            {0x00,0x41,0x22,0x14,0x08}, // >
            {0x02,0x01,0x51,0x09,0x06}, // ?
            {0x32,0x49,0x79,0x41,0x3E}, // @
            {0x7E,0x11,0x11,0x11,0x7E}, // A
            {0x7F,0x49,0x49,0x49,0x36}, // B
            {0x3E,0x41,0x41,0x41,0x22}, // C
            {0x7F,0x41,0x41,0x22,0x1C}, // D
            {0x7F,0x49,0x49,0x49,0x41}, // E
            {0x7F,0x09,0x09,0x09,0x01}, // F
            {0x3E,0x41,0x49,0x49,0x7A}, // G
            {0x7F,0x08,0x08,0x08,0x7F}, // H
            {0x00,0x41,0x7F,0x41,0x00}, // I
            {0x20,0x40,0x41,0x3F,0x01}, // J
            {0x7F,0x08,0x14,0x22,0x41}, // K
            {0x7F,0x40,0x40,0x40,0x40}, // L
            {0x7F,0x02,0x0C,0x02,0x7F}, // M
            {0x7F,0x04,0x08,0x10,0x7F}, // N
            {0x3E,0x41,0x41,0x41,0x3E}, // O
            {0x7F,0x09,0x09,0x09,0x06}, // P
            {0x3E,0x41,0x51,0x21,0x5E}, // Q
            {0x7F,0x09,0x09,0x19,0x66}, // R
            {0x26,0x49,0x49,0x49,0x32}, // S
            {0x01,0x01,0x7F,0x01,0x01}, // T
            {0x3F,0x40,0x40,0x40,0x3F}, // U
            {0x1F,0x20,0x40,0x20,0x1F}, // V
            {0x3F,0x40,0x38,0x40,0x3F}, // W
            {0x63,0x14,0x08,0x14,0x63}, // X
            {0x07,0x08,0x70,0x08,0x07}, // Y
            {0x61,0x51,0x49,0x45,0x43}, // Z
            {0x00,0x7F,0x41,0x41,0x00}, // [
            {0x02,0x04,0x08,0x10,0x20}, // backslash
            {0x00,0x41,0x41,0x7F,0x00}, // ]
            {0x04,0x02,0x01,0x02,0x04}, // ^
            {0x40,0x40,0x40,0x40,0x40}, // _
            {0x00,0x01,0x02,0x04,0x00}, // `
            {0x20,0x54,0x54,0x54,0x78}, // a
            {0x7F,0x48,0x44,0x44,0x38}, // b
            {0x38,0x44,0x44,0x44,0x20}, // c
            {0x38,0x44,0x44,0x48,0x7F}, // d
            {0x38,0x54,0x54,0x54,0x18}, // e
            {0x08,0x7E,0x09,0x01,0x02}, // f
            {0x0C,0x52,0x52,0x52,0x3E}, // g
            {0x7F,0x08,0x04,0x04,0x78}, // h
            {0x00,0x44,0x7D,0x40,0x00}, // i
            {0x20,0x40,0x44,0x3D,0x00}, // j
            {0x7F,0x10,0x28,0x44,0x00}, // k
            {0x00,0x41,0x7F,0x40,0x00}, // l
            {0x7C,0x04,0x18,0x04,0x78}, // m
            {0x7C,0x08,0x04,0x04,0x78}, // n
            {0x38,0x44,0x44,0x44,0x38}, // o
            {0x7C,0x14,0x14,0x14,0x08}, // p
            {0x08,0x14,0x14,0x18,0x7C}, // q
            {0x7C,0x08,0x04,0x04,0x08}, // r
            {0x48,0x54,0x54,0x54,0x20}, // s
            {0x04,0x3F,0x44,0x40,0x20}, // t
            {0x3C,0x40,0x40,0x20,0x7C}, // u
            {0x1C,0x20,0x40,0x20,0x1C}, // v
            {0x3C,0x40,0x30,0x40,0x3C}, // w
            {0x44,0x28,0x10,0x28,0x44}, // x
            {0x0C,0x50,0x50,0x50,0x3C}, // y
            {0x44,0x64,0x54,0x4C,0x44}, // z
            {0x00,0x08,0x36,0x41,0x00}, // {
            {0x00,0x00,0x7F,0x00,0x00}, // |
            {0x00,0x41,0x36,0x08,0x00}, // }
            {0x10,0x08,0x08,0x10,0x08}, // ~
        };

        if (c >= 32 && c <= 126) {
            return font5x7[c - 32];
        }
        // 未知字符返回空格
        return font5x7[0];
    }
};

#endif // DAC_CONTROLLER_H
