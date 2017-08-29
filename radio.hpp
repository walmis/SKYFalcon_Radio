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

class Radio : public RH_RF22 {
public:
	Radio() : RH_RF22((radio_sel::port<<5)|radio_sel::pin,
			(radio_irq::port<<5)|radio_irq::pin) {

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
