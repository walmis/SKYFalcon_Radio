/*
 * SwUart.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: walmis
 */

#include "SwUart.hpp"
#include <xpcc/architecture.hpp>
#include "pindefs.hpp"

using namespace xpcc;
using namespace lpc11;

enum {
	WAIT_START,
	DATA_RECV
};

SWUartRx::SWUartRx(uint32_t baud) : buffer(64) {
	setBaud(baud);

	lpc11::Timer32B0::init(lpc11::TimerMode::TIMER_MODE, 1,
			lpc11::PrescaleMode::PRESCALE_TICKVAL);
	lpc11::Timer32B0::enableCapturePins();
	setCapture(true);

	lpc11::Timer32B0::activate();

	lpc11::Timer32B0::enableIRQ();
}

void SWUartRx::setBaud(uint32_t baud) {
	_bitperiod = SystemCoreClock / 1000000UL * (1000000UL / baud);
}

void SWUartRx::_isr() {
	if (Timer32B0::getIntStatus(TmrIntType::CR0_INT)) {
		Timer32B0::clearIntPending(TmrIntType::CR0_INT);

		if (state == WAIT_START) {
			setCapture(false);

			tstamp = Timer32B0::getCaptureValue() + _bitperiod
					+ (_bitperiod >> 1);
			Timer32B0::configureMatch(0, tstamp, ExtMatchOpt::EXTMATCH_NOTHING,
					false, false, true);

			curr_bit = 0;
			_chr = 0;

			state = DATA_RECV;
		}

	}
	if (Timer32B0::getIntStatus(TmrIntType::MR0_INT)) {
		Timer32B0::clearIntPending(TmrIntType::MR0_INT);
		if(state == DATA_RECV){
			tstamp += _bitperiod;

			if (curr_bit == 8) { //stop bit
				state = WAIT_START;
				setCapture(true);
				if (sw_rx::read()) { //stop bit must be positive
					buffer.write(_chr);
				}
			} else {
				_chr |= (sw_rx::read() & 1) << curr_bit;

				Timer32B0::updateMatchValue(0, tstamp);
			}
			curr_bit++;
		}
	}

}

void SWUartRx::setCapture(bool en) {
	if (en) {
		Timer32B0::configCapture(
				(lpc11::CaptureFlags) (lpc11::CaptureFlags::FALLING_EDGE
						| lpc11::CaptureFlags::INT_ON_CAPTURE));
	} else {
		Timer32B0::configCapture(CaptureFlags::DISABLED); //disable capture
	}

}


SWUartRx sw_rx(115200);

extern "C" void TIMER32_0_IRQHandler() {
	sw_rx._isr();
}

