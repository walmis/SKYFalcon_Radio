/*
 * remote_control.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#include "remote_control.hpp"
#include  "Axes.hpp"

void RemoteControl::handleInit() {

	static bool hw_init = false;
	if(!hw_init) {
		xpcc::TickerTask::sleep(40);
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

	XPCC_LOG_DEBUG .printf("init radio f:%d txpw:%d fh:%d\n", freq, txPower, numFhChannels );

	txPacketTimer.restart(txInterval);

	setFHStepSize(10);
	setFrequency(freq, afcPullIn);
	setTxPower(txPower);
	setModemConfig(modemCfg);

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
	if(numFhChannels)
		setFHChannel((rcData.seq^0x55) % numFhChannels);

	busy_rx = false;
	setModeRx();

	dataEvent.signal();
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
			rxDataLen = 0;
		}

		dataEvent.signal();
	}
}

//called from irq thread context
void RemoteControl::handleRxStart() {
	ledGreen::set();
}

//called from irq thread context
void RemoteControl::handleReset() {
	XPCC_LOG_DEBUG << "RADIO RESET OCCURED\n";
	//reset = 1;
}

bool RemoteControl::clearChannel() {
	uint8_t status = ezmacStatusRead();
	if(!(status & RH_RF22_EZMAC_PKRX)) {
		ledGreen::reset();
		noiseFloor = ((uint16_t) noiseFloor * 31 + rssiRead()) / 32;
		return true;
	}
	return false;
}

void RemoteControl::handleTick() {
	//if(!radio_irq::read()) {
	//	XPCC_LOG_DEBUG << "isr missed\n";
		//RH_RF22::isr0();
	//}
	if(reset) {
		handleInit();
	}

	if (!transmitting()) {
		ledRed::reset();
		if(rxDataLen) {
			if(rxDataLen >= sizeof(Packet)) {
				Packet* p = (Packet*)packetBuf;

				switch(p->id) {

				case PACKET_DATA: {
					uint8_t size = rxDataLen - sizeof(Packet);
					//printf("recv %d\n", size);
					if(rxbuf.bytes_free() < size) {
						printf("drop packet\n");
					} else {
						if(p->ackSeq == rcData.seq) {
							rxbuf.write(packetBuf+sizeof(Packet), size);
						} else {
							printf("discard duplicate seq:%d ackSeq:%d\n", rcData.seq, p->ackSeq);
						}

					}
				}
				break;

				}

				if(p->id >= PACKET_RC) {
					//printf("rx seq %d ackseq %d\n", p->seq, p->ackSeq);
					//check for lost packets
					if((uint8_t)(lastSeq+1) != p->seq) {
						if(p->seq < lastSeq) {
							_rxBad += 255-lastSeq + p->seq;
						} else {
							_rxBad += p->seq - (lastSeq+1);
						}
					}

					remRssi = p->rssi;
					remNoise = p->noise;

					rcData.ackSeq = p->seq; //acknowledge packet
					lastSeq = p->seq;
					lastAckSeq = p->ackSeq;

					if(rcData.seq == lastAckSeq) {
						//our packet was acknowledged
						txPacketTimer.restart(0);
					}
				}
			}

			//XPCC_LOG_DEBUG.dump_buffer(buf, len);
		}

		if (!sending && txPacketTimer.isExpired() && clearChannel()) {

			bool noAck = false;
			//sent packet was not acknowledged
			if(rcData.seq != lastAckSeq) {
				//if(rcData.seq == lastAckSeq)
				//if packet was lost, restore previous FH channel
				if(numFhChannels)
					setFHChannel(((rcData.seq-1)^0x55) % numFhChannels);
				noAck = true;
				_txBad++;
			}

			rcData.seq++;
			for(int i = 0; i < 16; i++) {
				rcData.channels[i] = axes.getChannel(i);
			}

			if(configurationChanged) {
				RadioCfgPacket p;
				p.seq = rcData.seq;
				p.ackSeq = rcData.ackSeq;
				p.frequency = getFreq();
				p.afcPullIn = getAfcPullIn();
				p.fhChannels = getNumFhChannels();
				p.modemCfg = getModemCfg();
				p.txPower = getTxPower();

				send((uint8_t*) (&p), sizeof(RadioCfgPacket));
				waitPacketSent();

				setModeIdle();

				//xpcc::sleep(100);

				setFrequency(freq, afcPullIn);
				setModemConfig(modemCfg);

				setModeRx();

				printf("sending radio conf\n");
				newRadioDataSent = true;
				configurationChanged = false;
			} else {
				if(noAck && dataLen) {
					//XPCC_LOG_DEBUG << "retry\n";
					//update rc data, and retransmit payload
					memcpy(packetBuf, (void*)&rcData, sizeof(RCPacket));
					send(packetBuf, dataLen);
					num_retries++;
					if(num_retries > max_retries) {
						printf("Max retries reached, drop packet\n");
						num_retries = 0;
						dataLen = 0;
					}

				} else {
					size_t avail = txbuf.bytes_used();
					if(avail) {
						size_t space = sizeof(packetBuf) - sizeof(RCPacket);
						uint8_t* ptr = packetBuf + sizeof(RCPacket);
						//copy pending rc packet to tx buffer
						memcpy(packetBuf, (void*)&rcData, sizeof(RCPacket));
						while(space && avail) {
							*ptr++ = txbuf.read();
							space--;
							avail--;
						}
						dataLen = ptr - packetBuf;

						printf("send %d\n", dataLen-sizeof(RCPacket));
						send(packetBuf, dataLen);

					} else {
						dataLen = 0;

						send((uint8_t*) (&rcData), sizeof(rcData));
					}

				}
			}
			ledRed::set();
			txPacketTimer.restart(txInterval);
		}

	}

}

void RemoteControl::handleInterrupt() {
	irqEvent.signal();
}

