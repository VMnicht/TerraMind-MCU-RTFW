#ifndef DEBUG_PRINTER_H
#define DEBUG_PRINTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 C 兼容的头文件或声明 C 风格的函数 */
#include <stdarg.h>
#include <stdio.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "../BSP/Serial_Device.h"

/**
 * @brief DebugPrinter 类，派生自 SerialDevice
 * 实现类似 C 语言 printf 的功能，用于实时打印调试信息。
 */
class DebugPrinter : public SerialDevice
{
public:
    /**
     * @brief 构造函数
     * @param huartx UART 句柄指针
     */
    DebugPrinter(UART_HandleTypeDef *huartx);

    /**
     * @brief 格式化打印函数
     * @param format 格式化字符串
     * @param ... 可变参数
     * @return true 发送成功, false 发送失败
     */
    bool printf(const char *format, ...);

private:
    static const uint16_t DEBUG_BUFFER_SIZE = 128; // 调试打印缓冲区大小
    char debug_buffer[DEBUG_BUFFER_SIZE];          // 缓冲区
};

#endif // __cplusplus

#endif // DEBUG_PRINTER_H
