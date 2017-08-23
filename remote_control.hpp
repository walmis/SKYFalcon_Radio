/*
 * remote_control.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef REMOTE_CONTROL_HPP_
#define REMOTE_CONTROL_HPP_

#include "radio.hpp"
#include "eeprom/eeprom.hpp"

enum PacketType {
	PACKET_RC = 100,
	PACKET_RF_PARAM_SET,
	PACKET_DATA_FIRST, //first data fragment
	PACKET_DATA, //data fragment
	PACKET_DATA_LAST, //last data fragment,
};

struct Packet {
	uint8_t id = PACKET_RC;
	uint8_t seq; //sequence number
	uint8_t ackSeq; //rx acknowledged seq number
	int8_t rssi; // local rssi dBm
	int8_t noise; //local noise dBm
} __attribute__((packed));

struct RadioCfgPacket : Packet {
	RadioCfgPacket() {
		id = PACKET_RF_PARAM_SET;
	}
	uint32_t frequency;
	uint32_t afcPullIn;
	uint8_t modemCfg;
	uint8_t fhChannels;
	uint8_t txPower;
} __attribute__((packed));

struct RCPacket : Packet{
	int16_t channels[16];
} __attribute__((packed));

class RemoteControl final : public Radio, public BufferedIODevice {
public:
	RemoteControl() :
		BufferedIODevice(256, 256),
		txPacketTimer(20),
		dataLen(0),
		num_retries(0)
	{}

	RCPacket rcData;

	uint8_t getLinkQuality();

	inline uint32_t getTxBad() {
		return _txBad;
	}

	uint32_t getAfcPullIn() const {
		return afcPullIn;
	}

	void setAfcPullIn(uint32_t afcPullIn) {
		if(this->afcPullIn != afcPullIn) {
			configurationChanged = true;
			this->afcPullIn = afcPullIn;
			eeprom.put(&EEData::afcPullIn, afcPullIn);
		}
	}

	bool clearChannel();

	uint32_t getFreq() const {
		return freq;
	}

	void setFreq(uint32_t freq) {
		if(freq != this->freq) {
			configurationChanged = true;
			eeprom.put(&EEData::rfFrequency, freq);
			this->freq = freq;
		}
	}

	RH_RF22::ModemConfigChoice getModemCfg() const {
		return modemCfg;
	}

	void setModemCfg(RH_RF22::ModemConfigChoice modemCfg) {
		if(modemCfg != this->modemCfg) {
			configurationChanged = true;
			eeprom.put(&EEData::modemCfg, modemCfg);
			this->modemCfg = modemCfg;
		}
	}

	uint8_t getNumFhChannels() const {
		return numFhChannels;
	}

	void setNumFhChannels(uint8_t numFhChannels) {
		if(numFhChannels != this->numFhChannels) {
			configurationChanged = true;
			eeprom.put(&EEData::fhChannels, numFhChannels);
			this->numFhChannels = numFhChannels;
		}
	}

	uint8_t getTxInterval() const {
		return txInterval;
	}

	void setTxInterval(uint8_t txInterval) {
		if(this->txInterval != txInterval) {
			eeprom.put(&EEData::txInterval, txInterval);
			txPacketTimer.restart(txInterval);
			this->txInterval = txInterval;
		}
	}

	uint8_t getTxPower() const {
		return txPower;
	}

	void setTxPower(uint8_t txPower) {
		if(this->txPower != txPower) {
			configurationChanged = true;
			this->txPower = txPower;
			eeprom.put(&EEData::txPower, txPower);
			RH_RF22::setTxPower(txPower);
		}
	}

	int8_t getNoiseFloor() {
		return (noiseFloor * 100 / 190) - 127;
	}

	int8_t getRssi() {
		return (rssi * 100 / 190) - 127;
	}

	int8_t getRemNoise() const {
		return remNoise;
	}

	int8_t getRemRssi() const {
		return remRssi;
	}

protected:
	friend class FreqConf;

	void handleTxComplete() override;
	void handleRxComplete() override;
	void handleRxStart() override;
	void handleReset() override;

	uint8_t noiseFloor;
	uint8_t rssi;

	int8_t remRssi;
	int8_t remNoise;

	void handleInit();
	void handleTick();

	bool sending;

	uint8_t lastSeq;
	uint8_t lastAckSeq;
	uint32_t _txBad;

	uint8_t packetBuf[255];
	uint8_t dataLen;

	Timeout<> txPacketTimer;

	///configuration parameters////
	uint32_t freq;
	uint32_t afcPullIn;
	uint8_t txPower;
	uint8_t txInterval;
	RH_RF22::ModemConfigChoice modemCfg;
	uint8_t numFhChannels; //Frequency hopping channels
	///////////////////////////////

	uint8_t num_retries;
	const uint8_t max_retries = 10;

	volatile bool busy_rx;

	bool configurationChanged;
	bool newRadioDataSent;

	volatile bool reset = 0;
	////////
};

extern RemoteControl radio;

#endif /* REMOTE_CONTROL_HPP_ */
