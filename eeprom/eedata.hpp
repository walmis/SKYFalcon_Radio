/*
 * eedata.hpp
 *
 *  Created on: May 8, 2014
 *      Author: walmis
 */

#ifndef EEDATA_HPP_
#define EEDATA_HPP_

#include "eeprom.hpp"
#include <xpcc/math.hpp>
#include <RH_RF22.h>

#define TOKEN 0x73

struct EEData {
	uint8_t token;

	uint32_t rfFrequency; //radio frequency
	uint32_t afcPullIn;
	uint8_t txPower;
	uint8_t fhChannels;
	uint8_t txInterval; //tx packet interval in ms
	RH_RF22::ModemConfigChoice modemCfg;

} __attribute((packed));

const EEData eeDefaults = {
		TOKEN,
		433000000,
		50000,
		RH_RF22_TXPOW_14DBM,
		0,
		60,
		RH_RF22::GFSK_Rb57_6Fd28_8,
};

#endif /* EEDATA_HPP_ */
