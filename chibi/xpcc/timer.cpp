/*
 * timer.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: walmis
 */

#include <xpcc/architecture.hpp>
#include <ch.h>

using namespace xpcc;
using namespace lpc11;

extern "C"
void port_timer_init() {
	Timer32B1::init(lpc11::TimerMode::TIMER_MODE, 48,
			lpc11::PrescaleMode::PRESCALE_TICKVAL);

	Timer32B1::activate();
	Timer32B1::enableIRQ();
}

extern "C"
void TIMER32_1_IRQHandler() {
	CH_IRQ_PROLOGUE();

	if(Timer32B1::getIntStatus(TmrIntType::MR0_INT)) {
		chSysLockFromISR();
		Timer32B1::clearIntPending(TmrIntType::MR0_INT);
		chSysTimerHandlerI();
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

extern "C"
void port_timer_start_alarm(systime_t time) {
	Timer32B1::configureMatch(0, time, ExtMatchOpt::EXTMATCH_NOTHING, false, false, true);
}

extern "C"
systime_t port_timer_get_time() {
	return Timer32B1::getCounterValue();
}

extern "C"
systime_t port_timer_get_alarm(void) {
	return Timer32B1::getMatchValue(0);
}

extern "C"
void port_timer_set_alarm(systime_t time) {
	if(Timer32B1::getCounterValue() > time) {
		//XPCC_LOG_DEBUG .printf("timer is in the future %d\n", time);
		//timer is in the future, probably due to debug
		//to prevent hang, rewind the timer
		Timer32B1::setCounterValue(time - 20);
	}
	Timer32B1::updateMatchValue(0, time);
}

extern "C"
void port_timer_stop_alarm() {
	Timer32B1::setInterruptOnMatch(0, false);
}
