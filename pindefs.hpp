/*
 * pindefs.hpp
 *
 *  Created on: May 6, 2013
 *      Author: walmis
 */

#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

#include <xpcc/architecture.hpp>

typedef xpcc::gpio::Invert<GPIO<0,0>> ledRed;
typedef xpcc::gpio::Invert<GPIO<0,1>> ledGreen;

typedef GPIO<0,1> progPin;
typedef GPIO<0,13> dbgPin;
typedef GPIO<0,17> sw_rx_pin;
typedef GPIO<0,21> usbConnect;
typedef GPIO<0,6> radio_irq;
typedef GPIO<0,7> radio_sel;

typedef GPIO<0,11> bt_key;
typedef GPIO<0,15> bt_rst;

typedef xpcc::lpc11::SpiMaster0 radioSpiMaster;

#endif /* PINDEFS_HPP_ */
