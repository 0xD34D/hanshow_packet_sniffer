#ifndef PRINTF_UART_H
#define PRINTF_UART_H

#ifdef printf
#undef printf
#endif

int printf(const char *fmt, ...);

#endif /* PRINTF_UART_H */
