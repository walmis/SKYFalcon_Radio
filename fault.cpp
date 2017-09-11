/*
 * fault.cpp
 *
 *  Created on: Sep 9, 2017
 *      Author: walmis
 */


#include "dbg_uart.h"
#include <stdint.h>
#include <LPC11Uxx.h>

enum { r0, r1, r2, r3, r12, lr, pc, psr};

extern "C" void HardFault_Handler(void) __attribute__((naked));
extern "C" void HardFault_Handler(void) //__attribute__((naked))
{
  //asm volatile("  TST LR, #4; ");
  register long lr asm("lr");
  if(lr != 0xFFFFFFF1U) {
	  asm("mrs r0, msp");
  } else {
	  asm("mrs r0, psp");
  }
  //asm("mov sp, r0");
  //asm("bkpt");


//				  ITE EQ;  \
//				  MRSEQ R0, MSP;  \
//				  MRSNE R0, PSP; \
//		       	  B Hard_Fault_Handler;");
  asm("B Hard_Fault_Handler");
	//__get
}

extern "C"
void Hard_Fault_Handler(uint32_t stack[]) __attribute((used));
extern "C"
void Hard_Fault_Handler(uint32_t stack[]) {

	//register uint32_t* stack = (uint32_t*)__get_MSP();

//////
	uart_print("Hard Fault\n");
////
	uart_print("\tr0  = 0x"); uart_put_hex( stack[r0]); uart_print("\n");
	uart_print("\tr1  = 0x"); uart_put_hex( stack[r1]); uart_print("\n");
	uart_print("\tr2  = 0x"); uart_put_hex( stack[r2]); uart_print("\n");
	uart_print("\tr3  = 0x"); uart_put_hex( stack[r3]); uart_print("\n");
	uart_print("\tr12 = 0x"); uart_put_hex( stack[r12]); uart_print("\n");
	uart_print("\tlr  = 0x"); uart_put_hex( stack[lr]); uart_print("\n");
	uart_print("\tpc  = 0x"); uart_put_hex( stack[pc]); uart_print("\n");
	uart_print("\tpsr = 0x"); uart_put_hex( stack[psr]); uart_print("\n");
//
//	LPC_PMU->GPREG3 |= 1;
//	NVIC_SystemReset();
	while(1) {}
}


extern "C" __attribute__((naked))
void WDT_IRQHandler() {
	  register long lr asm("lr");
	  if(lr != 0xFFFFFFF1U) {
		  asm("mrs r0, psp");
	  } else {
		  asm("mrs r0, msp");
	  }
	  //asm("mov sp, r0");
	  //asm("bkpt");


	//				  ITE EQ;  \
	//				  MRSEQ R0, MSP;  \
	//				  MRSNE R0, PSP; \
	//		       	  B Hard_Fault_Handler;");
	  asm("B WDT_Handler");
}

extern "C" __attribute((used))
void WDT_Handler(uint32_t stack[])  {
	LPC_WWDT->MOD |= (1<<3);
	uart_print("WDT Event\n");
////
	uart_print("\tr0  = 0x"); uart_put_hex( stack[r0]); uart_print("\n");
	uart_print("\tr1  = 0x"); uart_put_hex( stack[r1]); uart_print("\n");
	uart_print("\tr2  = 0x"); uart_put_hex( stack[r2]); uart_print("\n");
	uart_print("\tr3  = 0x"); uart_put_hex( stack[r3]); uart_print("\n");
	uart_print("\tr12 = 0x"); uart_put_hex( stack[r12]); uart_print("\n");
	uart_print("\tlr  = 0x"); uart_put_hex( stack[lr]); uart_print("\n");
	uart_print("\tpc  = 0x"); uart_put_hex( stack[pc]); uart_print("\n");
	uart_print("\tpsr = 0x"); uart_put_hex( stack[psr]); uart_print("\n");

	//while(1);
}

