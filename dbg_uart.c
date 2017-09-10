/*
 * dbg_uart.c
 *
 *  Created on: Sep 9, 2017
 *      Author: walmis
 */
#include <stdint.h>
#include <string.h>
#include <LPC11Uxx.h>

#define LSR_RDR         (0x01<<0)
#define LSR_OE          (0x01<<1)
#define LSR_PE          (0x01<<2)
#define LSR_FE          (0x01<<3)
#define LSR_BI          (0x01<<4)
#define LSR_THRE        (0x01<<5)
#define LSR_TEMT        (0x01<<6)
#define LSR_RXFE        (0x01<<7)

char* ltoa( long value, char *string, int radix )
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v;
  int sign;
  char *sp;

  if ( string == NULL )
  {
    return 0 ;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0 ;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
  {
    v = -value;
  }
  else
  {
    v = (unsigned long)value;
  }

  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}

void uart_send(const char *BufferPtr, unsigned Length)
{
  while ( Length != 0 )
  {
	  while ( !(LPC_UART->LSR & LSR_THRE) );
	  LPC_UART->THR = *BufferPtr;
      BufferPtr++;
      Length--;
  }
  return;
}

void uart_put_hex(int i) {
	char str[32];
	ltoa(i, str, 16);
	uart_send(str, strlen(str));
}

void uart_put_hex_byte(char i) {
	char str[32];
	ltoa(i, str, 16);
	if(str[1] == '\0') {
		str[2] = '\0';
		str[1] = str[0];
		str[0] = '0';
	}
	uart_send(str, strlen(str));
}

void uart_put_dec(int i) {
	char str[32];
	ltoa(i, str, 10);
	uart_send(str, strlen(str));
}

void uart_print(const char* str) {
	uart_send(str, strlen(str));
}

void uart_print_buf(const char* buf, int len) {
	int i = 0;
	while(len--) {
		uart_put_hex_byte(*buf++); uart_print(" ");
		i++;
		if(i >= 16) { uart_print("\n"); i = 0; }
	}
	uart_print("\n");
}


