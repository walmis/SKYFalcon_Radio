#include <xpcc/processing/rtos_abstraction.hpp>
#include <ch.h>
#include "../../dbg_uart.h"

namespace xpcc {
  IRQWrapper::IRQWrapper(void *lr) {
    CH_IRQ_PROLOGUE();
    //uart_print("lr "); uart_put_hex((int)lr);
    //uart_print("\n");
    data = (void*)lr;
  }
  //epilogue
  IRQWrapper::~IRQWrapper() {
    regarm_t _saved_lr = (regarm_t)data;
    CH_IRQ_EPILOGUE();
  }
}
