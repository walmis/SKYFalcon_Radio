/*
 * dbg_uart.h
 *
 *  Created on: Sep 9, 2017
 *      Author: walmis
 */

#ifndef DBG_UART_H_
#define DBG_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

void uart_send(const char *BufferPtr, unsigned Length);
void uart_put_hex(int i);
void uart_put_hex_byte(char i);
void uart_put_dec(int i);
void uart_print(const char* str);
void uart_print_buf(const char* buf, int len);

#ifdef __cplusplus
}
#endif

#endif /* DBG_UART_H_ */
