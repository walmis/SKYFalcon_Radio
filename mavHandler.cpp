/*
 * mavHandler.cpp
 *
 *  Created on: Oct 13, 2014
 *      Author: walmis
 */

#include <mavHandler.hpp>
#include <remote_control.hpp>

mavlink_system_t mavlink_system;

#include "packets.hpp"

using namespace xpcc;

MAVHandler::MAVHandler(uint8_t sysid) : sysid(sysid) {
	radio = 0;
	bluetooth = 0;
	uart = 0;
	usb = 0;
	mavlink_system.sysid = sysid;
	mavlink_system.compid = 1;
}

void MAVHandler::handleTick() {
	if(radio == 0) return;

	int16_t c;

	//forward data from radio to USB and RadioTX(with bluetooth tied)
	//Radio -> uart(bluetooth), usb
	if(radio->rxAvailable()) {

		uint8_t avail = radio->rxAvailable();
		while(avail) {
			c = radio->read();

			if(uart) uart->write(c);
			if(usb)	usb->write(c);

			parseCharFromRadio(c);
			avail--;
		}
	}

	//todo parse radio config packets from usb, uart, bluetooth

    //forward bluetoothrx -> radio
	if(bluetooth) {
		uint8_t avail = bluetooth->rxAvailable();
		while(avail) {
			c = bluetooth->read();
			radio->write(c);
			parseCharLocal(CHAN_BLUETOOTH, c);
			avail--;
		}
	}

	//forward uart -> radio
	if(uart) {
		uint8_t avail = uart->rxAvailable();

		while(avail) {
			c = uart->read();
			radio->write(c);

			parseCharLocal(CHAN_UART, c);
			avail--;
		}
	}

	//forward usb -> radio
	if(usb) {
		uint8_t avail = usb->rxAvailable();
		while(avail) {
			c = usb->read();
			radio->write(c);

			parseCharLocal(CHAN_USB, c);
			avail--;
		}
	}

	static xpcc::Timeout<> heartBeatTimer(1000);
	static xpcc::Timeout<> statusUpdateTimer(200);
	static xpcc::Timeout<> messageTimeout(1000);

	//make sure we are not in a middle of a mavlink message before injecting our messages
	mavlink_status_t* status = mavlink_get_channel_status(CHAN_RADIO);
	if(messageTimeout.isExpired() || status->parse_state <= MAVLINK_PARSE_STATE_IDLE) {
		messageTimeout.restart(1000);
		if(heartBeatTimer.isExpired()) {
			sendRadioConf(CHAN_UART);

			if(radio->txAvailable() >= (MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES)) {
				mavlink_msg_heartbeat_send(CHAN_RADIO, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
						MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);

				heartBeatTimer.restart(1000);
			}
		}

		if(statusUpdateTimer.isExpired()) {
			sendRadioStatus(CHAN_USB);
			sendRadioStatus(CHAN_UART);
			statusUpdateTimer.restart(200);
		}
	}

}

void MAVHandler::sendRadioStatus(mavlink_channel_t chan) {
	union {
		RadioStatus st;
		uint8_t data[16];
	};
	memset(data, 0, 16);
	st.noise = radio->getNoiseFloor();
	st.rssi = radio->getRssi();
	st.txGood = radio->getTxGood();
	st.txBad = radio->getTxBad();
	st.rxGood = radio->getRxGood();
	st.rxBad = radio->getRxBad();
	st.remNoise = radio->getRemNoise();
	st.remRssi = radio->getRemRssi();

	mavlink_system.sysid = remSysid;
	mavlink_msg_data16_send(chan, RADIO_STATUS, sizeof(RadioStatus), data);
	mavlink_system.sysid = sysid;

	mavlink_msg_radio_send(chan,
			((radio->getRssi()+127)*190UL)/100,
			((radio->getRemRssi()+127)*190UL)/100,
			radio->txAvailable(),
			((radio->getNoiseFloor()+127)*190UL)/100,
			((radio->getRemNoise()+127)*190UL)/100,
			radio->getRxBad(), 0);

}

void MAVHandler::sendRadioConf(mavlink_channel_t chan) {
	union {
		RadioConfiguration st;
		uint8_t data[16];
	};
	memset(data, 0, 16);
	st.fhop_channels = radio->getNumFhChannels();
	st.frequency = radio->getFreq();
	st.txInterval = radio->getTxInterval();
	st.modemCfg = radio->getModemCfg();
	st.afc = radio->getAfcPullIn();
	st.txPower = radio->getTxPower();

	mavlink_system.sysid = remSysid;
	mavlink_msg_data16_send(chan, CURRENT_RADIO_CONFIGURATION, sizeof(RadioStatus), data);

	mavlink_system.sysid = sysid;
}

void MAVHandler::parseCharLocal(mavlink_channel_t chan, uint8_t c) {
	mavlink_message_t msg;
	mavlink_status_t status;
	if(mavlink_parse_char(chan, c, &msg, &status)) {
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_DATA16:
			uint8_t type = mavlink_msg_data16_get_type(&msg);
			if(type == SET_RADIO_CONFIGURATION) {
				uint8_t buf[16];
				mavlink_msg_data16_get_data(&msg, buf);

				RadioConfiguration* cfg = (RadioConfiguration*)buf;
				//XPCC_LOG_DEBUG .printf("set radio configuration\n");
				//XPCC_LOG_DEBUG .printf("F %d\n", cfg->frequency);

				radio->setFreq(cfg->frequency);
				radio->setAfcPullIn(cfg->afc);
				radio->setNumFhChannels(cfg->fhop_channels);
				radio->setModemCfg((RH_RF22::ModemConfigChoice)cfg->modemCfg);
				radio->setTxPower(cfg->txPower);
				radio->setTxInterval(cfg->txInterval);

				sendRadioConf(chan);
			}

		break;
		}
	}
}

void MAVHandler::parseCharFromRadio(uint8_t c) {
	mavlink_message_t msgBuf;
	if(mavlink_parse_char(CHAN_RADIO, c, &msgBuf, &mavStatus)) {
//		printf("Received message with ID %d, sequence: %d from component %d of system %d\n",
//				msgBuf.msgid, msgBuf.seq, msgBuf.compid, msgBuf.sysid);
		switch(msgBuf.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			mavlink_msg_heartbeat_decode(&msgBuf, &heartBeat);

			if(lastHeartbeat == 0 || !getLinkStatus()) {
				//comms restored

				remSysid = msgBuf.sysid;
				printf("requesting data stream from sysid:%d\n", remSysid);

				mavlink_msg_request_data_stream_send(CHAN_RADIO, remSysid,
						0, MAV_DATA_STREAM_EXTRA1, 5, 1);
				mavlink_msg_request_data_stream_send(CHAN_RADIO, remSysid,
						0, MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1);
				mavlink_msg_request_data_stream_send(CHAN_RADIO, remSysid,
						0, MAV_DATA_STREAM_EXTRA2, 5, 1);
				mavlink_msg_request_data_stream_send(CHAN_RADIO, remSysid,
						0, MAV_DATA_STREAM_EXTRA3, 2, 1);
			}

			lastHeartbeat = xpcc::Clock::now();
			break;

		}
	}
}

bool MAVHandler::getLinkStatus() {
	return (xpcc::Clock::now() - lastHeartbeat) < 2000;
}

void MAVHandler::send_chan(mavlink_channel_t chan, const uint8_t* buffer, uint8_t len) {
	switch(chan) {
	case CHAN_RADIO:
		if(radio) radio->write(buffer, len);
		break;
	case CHAN_BLUETOOTH:
		break;
	case CHAN_UART:
		if(uart) uart->write(buffer, len);
		break;
	case CHAN_USB:
		if(usb) usb->write(buffer, len);
		break;
	}
}

void comm_send_buffer(mavlink_channel_t chan, const uint8_t* buffer, uint8_t len) {
	mavHandler.send_chan(chan, buffer, len);
}
