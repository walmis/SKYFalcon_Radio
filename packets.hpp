/*
 * packets.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: walmis
 */

#ifndef PACKETS_HPP_
#define PACKETS_HPP_

#include "mavlink/protocol.h"

enum PacketId {
	CURRENT_RADIO_CONFIGURATION,
	SET_RADIO_CONFIGURATION,
	RADIO_STATUS
};

struct RadioConfiguration {
	uint32_t frequency;
	uint32_t afc;
	uint16_t txInterval;
	uint8_t fhop_channels;
	uint8_t modemCfg;
	uint8_t txPower;
} __attribute((packed));

struct RadioStatus {
	int8_t rssi;
	int8_t noise;
	int8_t remRssi;
	int8_t remNoise;
	uint16_t txGood;
	uint16_t txBad;
	uint16_t rxGood;
	uint16_t rxBad;
} __attribute((packed));


//void unpackRadioConfiguration(mavlink_data16_t &packet, RadioConfiguration& cfg) {
//	memcpy((uint8_t*)&cfg, packet.data, sizeof(RadioConfiguration));
//}
//
//void unpackRadioStatus(mavlink_data16_t &packet, RadioStatus& status) {
//	memcpy((uint8_t*)&status, packet.data, sizeof(RadioStatus));
//}
//
//void packRadioConfiguration(mavlink_data16_t &packet, RadioConfiguration& cfg, bool set) {
//	packet.type = set?SET_RADIO_CONFIGURATION:CURRENT_RADIO_CONFIGURATION;
//	packet.len = sizeof(RadioConfiguration);
//	memcpy(packet.data, (uint8_t*)&cfg, sizeof(RadioConfiguration));
//}
//
//void packRadioStatus(mavlink_data16_t &packet, RadioStatus &status) {
//	packet.type = RADIO_STATUS;
//	packet.len = sizeof(RadioStatus);
//	memcpy(packet.data, (uint8_t*)&status, sizeof(RadioStatus));
//	mavlink_msg_data16_send()
//}


#endif /* PACKETS_HPP_ */
