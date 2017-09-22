/*
 * remote_control.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#include "remote_control.hpp"
#include  "Axes.hpp"

class StopWatch {
public:
	StopWatch() {
		time = chVTGetSystemTimeX();
	}
	void start() {
		time = chVTGetSystemTimeX();
	}
	uint32_t stop() {
		return chVTGetSystemTimeX() - time;
	}
	uint32_t time;
};

void RemoteControl::initialize() {

	static bool hw_init = false;
	if(!hw_init) {
		xpcc::sleep(40);
		if(!this->RH_RF22::HWinit()) {
			panic("radio HW init fail");
		}
		hw_init = true;
	}


	if(!this->RH_RF22::init()) {
		panic("radio init fail");
	}

	eeprom.get(&EEData::rfFrequency, freq);
	eeprom.get(&EEData::afcPullIn, afcPullIn);
	eeprom.get(&EEData::txPower, txPower);
	eeprom.get(&EEData::modemCfg, modemCfg);
	eeprom.get(&EEData::txInterval, txInterval);
	eeprom.get(&EEData::fhChannels, numFhChannels);

	XPCC_LOG_DEBUG .printf("init radio f:%d txpw:%d fh:%d cfg:%d\n", freq, txPower, numFhChannels, modemCfg);

	txPacketTimer.restart(txInterval);

	setFHStepSize(10);
	setFrequency(freq, afcPullIn);
	setTxPower(txPower);
	setModemConfig(modemCfg);

	setModeRx();
}

void RemoteControl::mainTask() {
	mainThread = chThdGetSelfX();
	initialize();

	while(1) {
		chThdSleep(MS2ST(100));

		eventmask_t e = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_IMMEDIATE);
		if(e) {
			printf("ev pending %d\n", e);

			if(e & EventFlags::EVENT_RX_COMPLETE) {
				clearRxPacket();
			}
		}

		if(clearChannel()) {
			StopWatch s;
			printf("send rc...");
			bool res = sendRCData();
			int time = s.stop();
			if(!res) {
				printf("No ack (%dms)\n", time/1000);
			} else {
				printf("OK (%dms)\n", time/1000);
			}
		}


		//resetFifos();
		//if(!radio_irq::read()) {
		//	XPCC_LOG_DEBUG << "isr missed\n";
			//RH_RF22::isr0();
		//}
//		if(reset) {
//			handleInit();
//		}
//
//		if (!transmitting()) {
//			ledRed::reset();
//
//			if (!sending && txPacketTimer.isExpired() && clearChannel()) {
//
//				if(configurationChanged) {
//					RadioCfgPacket p;
//					p.frequency = getFreq();
//					p.afcPullIn = getAfcPullIn();
//					p.fhChannels = getNumFhChannels();
//					p.modemCfg = getModemCfg();
//					p.txPower = getTxPower();
//
//					send((uint8_t*) (&p), sizeof(RadioCfgPacket));
//					waitPacketSent();
//
//					setModeIdle();
//
//					//xpcc::sleep(100);
//
//					setFrequency(freq, afcPullIn);
//					setModemConfig(modemCfg);
//
//					setModeRx();
//
//					printf("sending radio conf\n");
//					newRadio//		if(reset) {
		//			handleInit();
		//		}
		//
		//		if (!transmitting()) {
		//			ledRed::reset();
		//
		//			if (!sending && txPacketTimer.isExpired() && clearChannel()) {
		//
		//				if(configurationChanged) {
		//					RadioCfgPacket p;
		//					p.frequency = getFreq();
		//					p.afcPullIn = getAfcPullIn();
		//					p.fhChannels = getNumFhChannels();
		//					p.modemCfg = getModemCfg();
		//					p.txPower = getTxPower();
		//
		//					send((uint8_t*) (&p), sizeof(RadioCfgPacket));
		//					waitPacketSent();
		//
		//					setModeIdle();
		//
		//					//xpcc::sleep(100);
		//
		//					setFrequency(freq, afcPullIn);
		//					setModemConfig(modemCfg);
		//
		//					setModeRx();
		//
		//					printf("sending radio conf\n");
		//					newRadioDataSent = true;
		//					configurationChanged = false;
		//				}
		//
		//				ledRed::set();
		//				txPacketTimer.restart(txInterval);
		//			}
		//
		//		}DataSent = true;
//					configurationChanged = false;
//				}
//
//				ledRed::set();
//				txPacketTimer.restart(txInterval);
//			}
//
//		}

	}
}

void RemoteControl::irqTask() {
	irqThread = chThdGetSelfX();

	while(1) {
		eventmask_t events = chEvtWaitAnyTimeout(IRQ_EVENT, MS2ST(100));

		if(radio_irq::read() == 0) {
			//uart_print("evt\n");
			this->RH_RF22::handleInterrupt();
		}
	}
}

uint8_t RemoteControl::getLinkQuality() {
	if(xpcc::Clock::now() - radio.getLastPreambleTime() > 250) {
		return 0;
	}
	int percent = 2 * (radio.lastRssi() + 100);
	if(percent > 100) percent = 100;
	if(percent < 0) percent = 0;
	return percent;
}

//called from irq thread context
void RemoteControl::handleTxComplete() {
	setModeRx();

	chEvtSignal(mainThread, (eventmask_t)EventFlags::EVENT_TX_COMPLETE);
}

void RemoteControl::mainTaskEntry(void* ths){
	static_cast<RemoteControl*>(ths)->mainTask();
}

void RemoteControl::irqTaskEntry(void* ths){
	static_cast<RemoteControl*>(ths)->irqTask();
}

//called from irq thread context
void RemoteControl::handleRxComplete() {
	busy_rx = false;
	rssi = (7*rssi + (uint8_t)lastRssi()) / 8;
	ledGreen::reset();
	if(available()) {
		if(rxDataLen) {
			XPCC_LOG_DEBUG .printf("packet not cleared\n");
		}
		rxDataLen = sizeof(packetBuf);
		if(!recv(packetBuf, &rxDataLen) ) {
			clearRxPacket();
		}

		chEvtSignal(mainThread, (eventmask_t)EventFlags::EVENT_RX_COMPLETE);
	}
}

//called from irq thread context
void RemoteControl::handleRxStart() {
	ledGreen::set();
	chEvtSignal(mainThread, (eventmask_t)EventFlags::EVENT_RX_START);
}

//called from irq thread context
void RemoteControl::handleReset() {
	XPCC_LOG_DEBUG << "RADIO RESET OCCURED\n";
	chEvtSignal(mainThread, (eventmask_t)EventFlags::EVENT_RADIO_RESET);
}

bool RemoteControl::clearChannel() {
	uint8_t status = ezmacStatusRead();
	if(!(status & RH_RF22_EZMAC_PKRX)) {
		noiseFloor = ((uint16_t) noiseFloor * 31 + rssiRead()) / 32;
		return true;
	}
	return false;
}

bool RemoteControl::sendRCData() {
	RCPacket p;
	for(int i = 0; i < 16; i++) {
		p.channels[i] = axes.getChannel(i);
	}
	rcState.seq++;
	setHeaderId(rcState.seq);

	uint8_t flags = (char)PacketFlags::PACKET_RC;

	if(txAvailable()) {
		flags |= (char)PacketFlags::FLAG_TELEM_PENDING;
	}

	setHeaderFlags(flags);
	//printf("seq %d\n", rcState.seq);

	send((uint8_t*)&p, sizeof(RCPacket));
	StopWatch s;
	s.start();
	if(!waitPacketSent()) {
		return false;
	}
	printf("tx(%dms)...", s.stop()/1000);

	bool ret;


	s.start();
	ret = waitRxStart(MS2ST(5));
	if(!ret){
		return false;
	}
	printf("st(%dms)...", s.stop()/1000);

	s.start();
	if(!waitRcACK(MS2ST(5))) {
		rcPacketsLost++;
		return false;
	}
	printf("rx(%dms)...", s.stop()/1000);

	return true;
}

bool RemoteControl::waitRcACK(systime_t timeout) {
	RxPacket p = waitRxPacket(timeout);
	if(p) {
		//printf("ack %d %d\n", rcState.seq, headerId());
		if(headerFlags() & PacketFlags::PACKET_RC_ACK) {
			if(headerId() == rcState.seq) {
				return true;
			}
		}
	}
	return false;
}

bool RemoteControl::waitPacketSent() {
	if(_mode == RHModeTx) {
		StopWatch s;
		ledRed::set();
		eventmask_t ev = chEvtWaitAnyTimeout((eventmask_t)EventFlags::EVENT_TX_COMPLETE, MS2ST(1000));
		ledRed::reset();
		if(!ev) {
			XPCC_LOG_DEBUG .printf("TX timeout!\n");
		}
		_lastTransmitTime = s.stop();
		return ev & EventFlags::EVENT_TX_COMPLETE;
	} else {
		return true;
	}
}

RemoteControl::RxPacket
RemoteControl::waitRxPacket(systime_t timeout) {
	eventmask_t evmask = chEvtWaitAnyTimeout((eventmask_t)EventFlags::EVENT_RX_COMPLETE, timeout);

	if(evmask & EventFlags::EVENT_RX_COMPLETE) {
		Packet* rx_packet = (Packet*)packetBuf;
		remRssi = rx_packet->rssi;
		remNoise = rx_packet->noise;

		return std::move(RxPacket(packetBuf, &rxDataLen));
	}

	return std::move(RxPacket(nullptr, nullptr));
}

bool RemoteControl::waitRxStart(systime_t timeout) {
	eventmask_t evmask = chEvtWaitAnyTimeout((eventmask_t)EventFlags::EVENT_RX_START, timeout);

	if(evmask & EventFlags::EVENT_RX_START) {
		return true;
	}

	return false;
}

bool RemoteControl::rxTelemetryPacket(systime_t timeout) {
	//wait for packet
	RxPacket p = waitRxPacket(timeout);
	if(p) {
		Packet* rx_packet = (Packet*)p.payload();
		if(rx_packet) {
			if(headerFlags() & PacketFlags::PACKET_TELEMETRY) {
				uint8_t size = rxDataLen - sizeof(Packet);
				uint8_t* payload = packetBuf+sizeof(Packet);
				//printf("recv %d\n", size);
				if(rxbuf.bytes_free() < size) {
					printf("drop packet\n");
				} else {
					if(headerId() == telemState.lastRxSeq) {
						rxbuf.write(payload, size);
					} else {
						printf("discard duplicate seq:%d ackSeq:%d\n", telemState.lastRxSeq, headerId());
					}

					telemState.lastRxSeq = headerId();

					//TODO: sendTelemACK(seq);
					return true;
				}
			}
		}
	}
	return false;
}

//void RemoteControl::updatePacketCounts(Packet* p) {
//	if(headerId() == PACKET_RC) { //if valid packet ID
//		//printf("rx seq %d ackseq %d\n", rx_packet->seq, p->ackSeq);
//		//check for lost packets
//		if((uint8_t)(lastSeq+1) != rx_packet->seq) {
//			if(rx_packet->seq < lastSeq) {
//				_rxBad += 255-lastSeq + rx_packet->seq;
//			} else {
//				_rxBad += rx_packet->seq - (lastSeq+1);
//			}
//		}
//
//		remRssi = p->rssi;
//		remNoise = p->noise;
//	}
//}
bool RemoteControl::waitTelemACK(systime_t timeout) {
	if(waitRxPacket(timeout)) {
		Packet* rx_packet = (Packet*)packetBuf;
		if(headerFlags() & PacketFlags::PACKET_TELEMETRY_ACK) {
			if(headerId() == telemState.seq) {
				return true;
			}
		}
	}
	return false;
}

void RemoteControl::sendTelemetryPacket() {
	//previous packet still loaded, meaning no ack was received
	//retransmit
	if(telemDataLen) {
		//XPCC_LOG_DEBUG << "retry\n";
		//update rc data, and retransmit payload
		setHeaderId(telemState.seq);
		setHeaderFlags((uint8_t)PacketFlags::PACKET_TELEMETRY);

		send(packetBuf, telemDataLen);

		num_retries++;
		if(num_retries > max_retries) {
			printf("[telemetry] Max retries reached, drop packet\n");
			num_retries = 0;
			telemDataLen = 0;
		}

	} else {
		size_t avail = txbuf.bytes_used();
		if(avail) {
			size_t space = sizeof(packetBuf) - sizeof(Packet);
			uint8_t* ptr = packetBuf + sizeof(Packet);

			while(space && avail) {
				*ptr++ = txbuf.read();
				space--;
				avail--;
			}

			telemDataLen = ptr - packetBuf;

			printf("[telemetry] send len:%d\n", telemDataLen-sizeof(Packet));

			setHeaderFlags((uint8_t)PacketFlags::PACKET_TELEMETRY);
			setHeaderId(telemState.seq++);

			send(packetBuf, telemDataLen);

		} else {
			telemDataLen = 0;
			return; //nothing to send, return
		}
	}

	waitPacketSent();

	if(waitTelemACK()) {
		telemDataLen = 0;
	}

}

//called from IRQ context
void RemoteControl::handleInterrupt() {
	chSysLockFromISR();
	chEvtSignalI(irqThread, IRQ_EVENT);
	chSysUnlockFromISR();
}

