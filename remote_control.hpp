/*
 * remote_control.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef REMOTE_CONTROL_HPP_
#define REMOTE_CONTROL_HPP_

#include <xpcc/processing/timeout.hpp>
#include "eeprom/eeprom.hpp"
#include "pindefs.hpp"
#include <ch.h>

#define IRQ_EVENT (1)
enum class PacketType {
	PACKET_RC = 100,
	PACKET_RF_PARAM_SET,
	PACKET_DATA_FIRST, //first data fragment
	PACKET_TELEMETRY, //data fragment
	PACKET_DATA_LAST, //last data fragment,
	RC_ACK
};

enum class PacketFlags {
	FLAG_TELEM_PENDING = 1<<0,
	PACKET_RC = 1<<1,
	PACKET_RC_ACK = 1<<2,
	PACKET_TELEMETRY = 1<<3,
	PACKET_TELEMETRY_ACK = 1<<4,
};

enum class EventFlags {
	EVENT_RX_START = 1<<0,
	EVENT_RX_COMPLETE = 1<<1,
	EVENT_TX_COMPLETE = 1<<2,
	EVENT_RADIO_RESET = 1<<8
};

ENUM_CLASS_FLAG(PacketFlags);
ENUM_CLASS_FLAG(EventFlags);

//
// Master------[ RCPacket seq=1]-----------------------------------------------------------[TelemACK seq=1]
//
// Slave ------------------------[ACK seq=1 FLAG_TELEM_PENDING][Telemetry len=64 seq=1]---

struct Packet {
	int8_t rssi; // local rssi dBm
	int8_t noise; //local noise dBm
} __attribute__((packed));

struct RadioCfgPacket : Packet {
	uint32_t frequency;
	uint32_t afcPullIn;
	uint8_t modemCfg;
	uint8_t fhChannels;
	uint8_t txPower;
} __attribute__((packed));

struct RCPacket : Packet{
	int16_t channels[16];
} __attribute__((packed));

struct RCAck {
	uint8_t seq; //sequence number
} __attribute__((packed));

class RemoteControl final : public RH_RF22, public BufferedIODevice {
public:
	RemoteControl() : RH_RF22(radio_sel::id, radio_irq::id),
		BufferedIODevice(txbuf, txbuf),
		txPacketTimer(20),
		dataLen(0),
		num_retries(0),
		sending(0)
	{
	}

	class RxPacket {
	public:
		RxPacket(uint8_t* ptr_data, uint8_t* ptr_len) :
		ptr_data(ptr_data), ptr_len(ptr_len)
		{}

		//indicates that the buffer is no longer used
		~RxPacket() { if(ptr_len) *ptr_len = 0; }

		uint8_t* payload() { return ptr_data; }
		uint8_t len() { if(ptr_len) return *ptr_len; else return 0; }

		operator bool() { return ptr_data; }

	private:
		uint8_t* ptr_data;
		uint8_t* ptr_len;
	};


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

    bool waitPacketSent() override;
    uint32_t lastTransmitTime() { return _lastTransmitTime; }

    void initialize();

    static void mainTaskEntry(void* ths);
    static void irqTaskEntry(void* ths);

    THD_WORKING_AREA(wa_main_thread, 512);
    THD_WORKING_AREA(wa_irq_thread, 512);

protected:
	friend class FreqConf;

	void mainTask();
	void irqTask();

	thread_t* mainThread;
	thread_t* irqThread;

	void handleTxComplete() override;
	void handleRxComplete() override;
	void handleRxStart() override;
	void handleReset() override;
	void handleInterrupt() override;

	bool sendRCData();
	bool waitRcACK(systime_t timeout = 0);
	bool waitTelemACK(systime_t timeout = 0);

	//wait for Rx start (preamble)
	bool waitRxStart(systime_t timeout);
	RxPacket waitRxPacket(systime_t timeout = 0);
	bool rxTelemetryPacket(systime_t timeout = 0);
	void sendTelemetryPacket();

	void clearRxPacket() { rxDataLen = 0; }

	struct {
		uint8_t seq; //sequence number
		uint8_t lastRxSeq; //last received packet's sequence number
		uint8_t lastAckSeq; //last acknowledge sequence number
	} rcState;

	struct {
		uint8_t seq; //sequence number
		uint8_t lastRxSeq; //last received packet's sequence number
		uint8_t lastAckSeq; //last acknowledge sequence number
	} telemState;

	uint8_t noiseFloor;
	uint8_t rssi;

	int8_t remRssi; //remote rssi dBm
	int8_t remNoise; //remote noise dBm



	bool sending;

	uint32_t _txBad;

	uint32_t rcPacketsLost;

	uint8_t telemDataLen;

	uint8_t packetBuf[255];
	uint8_t rxDataLen = 0;
	uint8_t dataLen = 0;

	uint32_t _lastTransmitTime;

	xpcc::Timeout<> txPacketTimer;

	///configuration parameters////
	uint32_t freq;
	uint32_t afcPullIn;
	uint8_t txPower;
	uint8_t txInterval;
	RH_RF22::ModemConfigChoice modemCfg;
	uint8_t numFhChannels; //Frequency hopping channels
	///////////////////////////////

	StaticIOBuffer<256> txbuf;
	StaticIOBuffer<256> rxbuf;

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
