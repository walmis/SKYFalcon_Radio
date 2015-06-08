/*
 * Axes.hpp
 *
 *  Created on: Mar 4, 2015
 *      Author: walmis
 */

#ifndef AXES_HPP_
#define AXES_HPP_

#include <xpcc/architecture.hpp>

#define NUM_CHANS 16

class Axes {
public:

	Axes();

	int16_t getChannel(uint8_t ch) {
		if(ch >= NUM_CHANS) return 0;

		return channels[ch];
	}

	void isr();
private:


	uint16_t channels[NUM_CHANS];
};

extern Axes axes;

#endif /* AXES_HPP_ */
