#include <xpcc/processing/rtos_abstraction.hpp>
#include <ch.h>

namespace xpcc {
  IRQWrapper::IRQWrapper() {
    CH_IRQ_PROLOGUE();
    data = (void*)_saved_lr;
  }
  //epilogue
  IRQWrapper::~IRQWrapper() {
    regarm_t _saved_lr = (regarm_t)data;
    CH_IRQ_EPILOGUE();
  }
}
