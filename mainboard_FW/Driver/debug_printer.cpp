#include "debug_printer.h"

/**
 * @brief 构造函数
 * @param huartx UART 句柄指针
 */
DebugPrinter::DebugPrinter(UART_HandleTypeDef *huartx) : SerialDevice(huartx)
{
    // 基类构造函数已处理 UART 句柄绑定
}

/**
 * @brief 格式化打印函数
 * 使用 vsnprintf 进行安全格式化，并调用基类的 SendString 发送。
 * @param format 格式化字符串
 * @param ... 可变参数
 * @return true 发送成功, false 发送失败
 */
bool DebugPrinter::printf(const char *format, ...)
{
    if (huart_ == nullptr || format == nullptr)
    {
        return false;
    }

    va_list args;
    va_start(args, format);

    // 使用 vsnprintf 将格式化字符串写入缓冲区，防止溢出
    int length = vsnprintf(debug_buffer, DEBUG_BUFFER_SIZE, format, args);

    va_end(args);

    if (length < 0)
    {
        return false; // 格式化错误
    }

    // 如果格式化后的长度超过缓冲区大小，vsnprintf 会截断并保证以 \0 结尾
    // 获取实际写入缓冲区的长度（不包括 \0）
    size_t actual_length = (length >= DEBUG_BUFFER_SIZE) ? (DEBUG_BUFFER_SIZE - 1) : (size_t)length;

    // 发送缓冲区中的数据
    return SendArray((uint8_t *)debug_buffer, (uint8_t)actual_length);
}
