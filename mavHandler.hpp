/*
 * mavHandler.hpp
 *
 *  Created on: Oct 13, 2014
 *      Author: walmis
 */

#ifndef MAVHANDLER_HPP_
#define MAVHANDLER_HPP_

#define MAVLINK_MAX_PAYLOAD_LEN 104
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_COMM_NUM_BUFFERS 4

#include "mavlink/mavlink_types.h"

#define CHAN_RADIO MAVLINK_COMM_0
#define CHAN_UART MAVLINK_COMM_1
#define CHAN_USB MAVLINK_COMM_2
#define CHAN_BLUETOOTH MAVLINK_COMM_3

extern mavlink_system_t mavlink_system;
extern void comm_send_buffer(mavlink_channel_t chan, const uint8_t* buffer, uint8_t len);

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)



#include <xpcc/architecture.hpp>
#include "remote_control.hpp"
//#include "mavlink/common/mavlink.h"
#include "mavlink/ardupilotmega/mavlink.h"

class MAVHandler final : xpcc::TickerTask {
public:
	MAVHandler(uint8_t sysid = 250);

	mavlink_heartbeat_t heartBeat;

	bool getLinkStatus();

	xpcc::IODevice* usb;
	xpcc::IODevice* bluetooth;
	xpcc::IODevice* uart;
	RemoteControl* radio;

	void send_chan(mavlink_channel_t channel, const uint8_t* data, uint8_t len);

protected:
	void handleTick();
	void parseCharFromRadio(uint8_t c);
	void parseCharLocal(mavlink_channel_t chan, uint8_t c);

	void sendRadioStatus(mavlink_channel_t chan);
	void sendRadioConf(mavlink_channel_t chan);

	mavlink_status_t mavStatus;

	uint8_t remSysid;
	uint8_t sysid;

	xpcc::Timestamp lastHeartbeat;
};

extern MAVHandler mavHandler;

#endif /* MAVHANDLER_HPP_ */
