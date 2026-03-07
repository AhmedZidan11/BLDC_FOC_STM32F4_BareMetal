#ifndef DRIVERS_LOG_H_
#define DRIVERS_LOG_H_

/**
 * @file log.h
 * @brief Minimal log output for STM32F4 (CMSIS only).
 *
 * Provides short text logs over USART2.
 *
 * Responsibilities:
 * - format log lines with timestamp, level and tag.
 * - send each line as one chunk via uart2 driver.
 *
 * @note Intended for main/application context, not ISR context.
 */

#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include "drivers/usart2.h"

/**
 * @brief Log severity level.
 *
 */
typedef enum {
	LOG_LEVEL_INFO  = 0,
	LOG_LEVEL_WARN  = 1,
	LOG_LEVEL_ERROR = 2
} log_level_t;

/**
 * @brief Initialize logger with uart handle.
 *
 * @param uart_h Pointer to USART2 handle used for log output.
 * @return true if pointers are valid, false otherwise.
 */
bool log_init(usart2_handle_t *uart_h);

/**
 * @brief Write one log line.
 *
 * Tag is a short module/event identifier used to simplify log filtering.
 * Typical examples in this project:
 * - APP : application/startup messages
 * - HB  : periodic heartbeat message
 *
 * Line format:
 * L,<time_ms>,<level>,<tag>,<message>\r\n
 *
 * @param level Log level.
 * @param tag Short module tag.
 * @param msg Fixed text message.
 */
void log_write(log_level_t level, const char *tag, const char *msg);

/**
 * @brief Write one formatted log line.
 *
 * Tag is a short module/event identifier used to simplify log filtering.
 *
 * Line format:
 * L,<time_ms>,<level>,<tag>,<message>\r\n
 *
 * @param level Log level.
 * @param tag Short module/event identifier.
 * @param fmt printf-style format string.
 */
void log_printf(log_level_t level, const char *tag, const char *fmt, ...);

/**
 * @brief Write one numeric data sample line.
 *
 * Data line format:
 * D,<time_ms>,<sample_id>,<v0>,<v1>,<v2>,<v3>\r\n
 *
 * Intended for machine-readable samples that are parsed later on PC side.
 *
 * @param sample_id Numeric identifier for sample type.
 * @param v0 Sample value 0.
 * @param v1 Sample value 1.
 * @param v2 Sample value 2.
 * @param v3 Sample value 3.
 */
void log_data_u32(uint32_t sample_id, uint32_t v0, uint32_t v1,
				  uint32_t v2, uint32_t v3);

/**
 * @brief Write info log line.
 *
 * @param tag Short module tag.
 * @param msg Fixed text message.
 */
#define LOGI(tag, msg) log_write(LOG_LEVEL_INFO, (tag), (msg))

/**
 * @brief Write warning log line.
 *
 * @param tag Short module/event identifier.
 * @param msg Fixed text message.
 */
#define LOGW(tag, msg) log_write(LOG_LEVEL_WARN, (tag), (msg))

/**
 * @brief Write error log line.
 *
 * @param tag Short module/event identifier.
 * @param msg Fixed text message.
 */
#define LOGE(tag, msg) log_write(LOG_LEVEL_ERROR, (tag), (msg))

/**
 * @brief Write formatted info log line.
 *
 */
#define LOGI_F(tag, fmt, ...) log_printf(LOG_LEVEL_INFO, (tag), (fmt), ##__VA_ARGS__)

/**
 * @brief Write formatted warning log line.
 *
 */
#define LOGW_F(tag, fmt, ...) log_printf(LOG_LEVEL_WARN, (tag), (fmt), ##__VA_ARGS__)

/**
 * @brief Write formatted error log line.
 *
 */






#endif
