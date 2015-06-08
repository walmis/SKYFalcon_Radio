/*
 * pindefs.hpp
 *
 *  Created on: May 6, 2013
 *      Author: walmis
 */

#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

#include <xpcc/architecture.hpp>

GPIO__OUTPUT(_ledRed, 0, 4);
GPIO__OUTPUT(_ledGreen, 0, 5);

typedef xpcc::gpio::Invert<_ledRed> ledRed;
typedef xpcc::gpio::Invert<_ledGreen> ledGreen;

GPIO__INPUT(progPin, 0, 1);
GPIO__IO(dbgPin, 0, 13);
GPIO__INPUT(sw_rx, 0, 17);

GPIO__OUTPUT(usbConnect, 0, 21);

GPIO__IO(radio_irq, 0, 6);
GPIO__IO(radio_sel, 0, 7);


typedef xpcc::lpc11u::SpiMaster0 radioSpiMaster;

#endif /* PINDEFS_HPP_ */
