/*
 * fault.cpp
 *
 *  Created on: Sep 9, 2017
 *      Author: walmis
 */


#include "dbg_uart.h"

extern "C"
void Hard_Fault_Handler(uint32_t stack[]) __attribute((used));
extern "C"
void Hard_Fault_Handler(uint32_t stack[]) {

	//register uint32_t* stack = (uint32_t*)__get_MSP();

//////
	uart_print("Hard Fault\n");
////
//	uart_print("r0  = 0x") stack[r0]);
//	uart_print("r1  = 0x") stack[r1]);
//	uart_print("r2  = 0x") stack[r2]);
//	uart_print("r3  = 0x") stack[r3]);
//	uart_print("r12 = 0x") stack[r12]);
//	uart_print("lr  = 0x") stack[lr]);
//	uart_print("pc  = 0x") stack[pc]);
//	uart_print("psr = 0x") stack[psr]);
//
//	LPC_PMU->GPREG3 |= 1;
//	NVIC_SystemReset();
	while(1) {}
}


extern "C" __attribute__((naked))
void WDT_IRQHandler() {
	  register long lr asm("lr");
	  if(lr & 4) {
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
	  asm("B WDT_Handler");
}

extern "C"
void WDT_Handler(uint32_t stack[]) {
	Uart1::init(115200);
	IODeviceWrapper<Uart1> d;
	IOStream w(d);

	w.printf("WDT\n");
////
	w.printf("r0  = 0x%08x\n", stack[r0]);
	w.printf("r1  = 0x%08x\n", stack[r1]);
	w.printf("r2  = 0x%08x\n", stack[r2]);
	w.printf("r3  = 0x%08x\n", stack[r3]);
	w.printf("r12 = 0x%08x\n", stack[r12]);
	w.printf("lr  = 0x%08x\n", stack[lr]);
	w.printf("pc  = 0x%08x\n", stack[pc]);
	w.printf("psr = 0x%08x\n", stack[psr]);

	while(1);
}

