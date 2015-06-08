/*
 * Axes.cpp
 *
 *  Created on: Mar 12, 2015
 *      Author: walmis
 */

#include "Axes.hpp"

using namespace xpcc;
using namespace xpcc::lpc11;

Axes::Axes() {
	Timer16B1::init(TimerMode::TIMER_MODE, 1, PrescaleMode::PRESCALE_USVAL);
	Timer16B1::configCapture((CaptureFlags)(CaptureFlags::FALLING_EDGE | CaptureFlags::INT_ON_CAPTURE));
	Timer16B1::enableCapturePins();

	Timer16B1::activate();
	Timer16B1::enableIRQ();
}

void Axes::isr() {
	static volatile uint16_t t_lastCapture;
	static volatile uint8_t _chan;
	static volatile bool _sync;

	if(Timer16B1::getIntStatus(TmrIntType::CR0_INT)) {
		uint16_t tcap = Timer16B1::getCaptureValue();
		Timer16B1::clearIntPending(TmrIntType::CR0_INT);

		//handle timer overflow
		if(t_lastCapture >= tcap) {
			_sync = false;
			_chan = 0;
			t_lastCapture = tcap;
			return;
		}

		uint16_t diff = (tcap - t_lastCapture);
		if(diff < 500) return;

		if(diff > 2500) {
			_sync = true;
			_chan = 0;
		} else {
			if(_sync && _chan < NUM_CHANS) {
				channels[_chan] = diff;
				_chan++;
			}
		}
		t_lastCapture = tcap;

	}
}

extern "C"
void TIMER16_1_IRQHandler() {
	axes.isr();
}
