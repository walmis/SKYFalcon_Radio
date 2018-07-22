
#include <xpcc/architecture.hpp>
#include <xpcc/debug.hpp>
#include <wirish.h>
#include <RHGenericSPI.h>
#include <stdarg.h>
#include "pindefs.hpp"
#include <stdio.h>
#include <ch.h>

int printf(const char* fmt, ...) {
	va_list ap;
	va_start(ap, fmt);

	XPCC_LOG_DEBUG .vprintf(fmt, ap);

	va_end(ap);

	return 0;
}

int puts(const char* s) {
	XPCC_LOG_DEBUG << s << xpcc::endl;
	return 1;
}

#ifdef putchar
#undef putchar
#endif

int putchar(int c) {
	XPCC_LOG_DEBUG << c;
	return 1;
}

void delay(uint32_t millis) {
	xpcc::sleep(millis);
}

void rh_yield() {
	xpcc::yield();
}

xpcc::RMutex mutex;

void rh_atomic_block_start() {
	//xpcc::GpioInt::disableInterrupts();
	//__disable_irq();
	mutex.lock();
}

void rh_atomic_block_end() {
	//xpcc::GpioInt::enableInterrupts();
	//__enable_irq();
	mutex.unlock();
}

uint32_t RH::millis() {
	return xpcc::Clock::now().getTime();
}
uint32_t RH::micros() {
	return chVTGetSystemTimeX();
}

void pinMode(uint8_t pin, WiringPinMode mode) {
	uint8_t port = pin>>5;
	uint8_t p = pin&0x1F;
	//printf("pinMode %d %d -> %d\n", port, pin, mode);

	switch(mode) {
	case WiringPinMode::OUTPUT:
		xpcc::lpc11::IOCon::setGpioMode(port, p);
		LPC_GPIO->DIR[port] |= (1<<p);
		break;
	case WiringPinMode::INPUT:
		xpcc::lpc11::IOCon::setGpioMode(port, p);
		LPC_GPIO->DIR[port] &= ~(1<<p);
		break;
	default:
		return;
	}
}

void attachInterrupt(uint8_t pin, void (*fn)(void), int mode) {
	xpcc::IntEdge edge =  xpcc::IntEdge::RISING_EDGE;
	if(mode == FALLING) {
		edge = xpcc::IntEdge::FALLING_EDGE;
	} else if(mode == CHANGE) {
		edge = (xpcc::IntEdge)(xpcc::IntEdge::RISING_EDGE | xpcc::IntEdge::FALLING_EDGE);
	}

	auto i = xpcc::GpioInt::attach(pin>>5, pin&0x1F,	fn, edge);
	i.leak();

}


void digitalWrite(uint8_t pin, uint8_t val) {
	uint8_t p = pin&0x1F;
	uint8_t port = pin>>5;
	//printf("DW %d %d -> %d\n", port, p, val);
	if(port == 0) {
		LPC_GPIO->W0[p] = val;
	} else if(port == 1) {
		LPC_GPIO->W1[p] = val;
	}

}

class Spi : RHGenericSPI {

public:
	void begin() {}
	void end() {}

	uint8_t transfer(uint8_t data) {
		//printf("spi write %02x\n", data);
		return radioSpiMaster::write(data);
	}
};

Spi hardware_spi;
