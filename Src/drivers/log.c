/**
 * @file log.c
 * @brief Minimal log output implementation.
 *
 */

#include "drivers/log.h"

#include "drivers/systick.h"
#include <stdio.h>

static usart2_handle_t *s_uart_h = NULL;

static char log_level_to_char(log_level_t level)
{
	switch (level)
	{
	case LOG_LEVEL_INFO:
		return 'I';

	case LOG_LEVEL_WARN:
		return 'W';

	case LOG_LEVEL_ERROR:
		return 'E';
	}

	return '?';
}

bool log_init(usart2_handle_t *uart_h)
{
	if (uart_h == NULL) return false;

	s_uart_h = uart_h;
	return true;
}

void log_printf(log_level_t level, const char *tag, const char *fmt, ...)
{
	if ((s_uart_h == NULL) || (tag == NULL) || (fmt == NULL)) return;

	uint32_t time_ms = SYSTICK_GetTimeMs();
	char level_char = log_level_to_char(level);

	char line[128];
	int len = snprintf(line,
					   sizeof(line),
					   "L,%lu,%c,%s,",
					   (unsigned long)time_ms,
					   level_char,
					   tag);

	if (len <= 0) return;

	size_t offset = (size_t)len;
	if (offset >= sizeof(line)) return;

	va_list args;
	va_start(args, fmt);
	int msg_len = vsnprintf(&line[offset], sizeof(line) - offset, fmt, args);
	va_end(args);

	if (msg_len <= 0) return;

	size_t used_len = offset + (size_t)msg_len;
	if (used_len >= (sizeof(line) - 2u))
	{
		used_len = sizeof(line) - 2u;
	}

	line[used_len++] = '\r';
	line[used_len++] = '\n';

	usart2_write(s_uart_h, (const uint8_t *)line, used_len);
}

void log_data_u32(uint32_t sample_id, uint32_t v0, uint32_t v1,
				  uint32_t v2, uint32_t v3)
{
	if (s_uart_h == NULL) return;

	uint32_t time_ms = SYSTICK_GetTimeMs();

	char line[128];
	int len = snprintf(line,
					   sizeof(line),
					   "D,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
					   (unsigned long)time_ms,
					   (unsigned long)sample_id,
					   (unsigned long)v0,
					   (unsigned long)v1,
					   (unsigned long)v2,
					   (unsigned long)v3);

	if (len <= 0) return;

	size_t write_len = (size_t)len;
	if (write_len > sizeof(line)) write_len = sizeof(line);
	usart2_write(s_uart_h, (const uint8_t *)line, write_len);
}

void log_write(log_level_t level, const char *tag, const char *msg)
{
	log_printf(level, tag, "%s", msg);
}

