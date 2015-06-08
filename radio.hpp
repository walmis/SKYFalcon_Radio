/*
 * radio.hpp
 *
 *  Created on: Sep 18, 2014
 *      Author: walmis
 */

#ifndef RADIO_HPP_
#define RADIO_HPP_

#include <xpcc/architecture.hpp>
#include "pindefs.hpp"
#include <RH_RF22.h>

extern void panic(const char* s);

using namespace xpcc;

class Radio : TickerTask, public RH_RF22 {
public:
	Radio() : RH_RF22((radio_sel::Port<<5)|radio_sel::Pin,
			(radio_irq::Port<<5)|radio_irq::Pin) {

	}

	void handleInit() {
		if(!HWinit()) {
			panic("radio init fail");
		}
	}

    inline uint16_t getRxBad() {
    	return _rxBad;
    }

    inline uint16_t getRxGood() {
    	return _rxGood;
    }

    inline uint16_t getTxGood() {
    	return _txGood;
    }

    bool transmitting() {
    	return mode() == RHModeTx;
    }

    bool idle() {
    	return mode() == RHModeIdle;
    }

private:

};


#endif /* RADIO_HPP_ */
