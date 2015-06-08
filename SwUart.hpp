#pragma once
/*
 * SwUart.hpp
 *
 *  Created on: Mar 3, 2015
 *      Author: walmis
 */

#include <xpcc/architecture.hpp>

class SWUartRx final : public xpcc::IODevice {
public:
	SWUartRx(uint32_t baud);

	void setBaud(uint32_t baud);
	void _isr();


	size_t write(char c) {
		return 0;
	};
	/// Read a single character
	int16_t read() {
		return buffer.read();
	}

	void flush() {};

	int16_t rxAvailable() {
		return buffer.bytes_used();
	}
	int16_t txAvailable() {
		return 0;
	}

private:
	void setCapture(bool en);

	IOBuffer buffer;

	uint8_t _chr;
	uint8_t curr_bit;
	uint8_t state;
	uint32_t _bitperiod;
	uint32_t tstamp;
};

extern SWUartRx sw_rx;

