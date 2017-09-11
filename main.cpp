#include <unistd.h>
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/debug.hpp>
#include "eeprom/eeprom.hpp"
#include "SwUart.hpp"
#include "Axes.hpp"
#include "mavHandler.hpp"
#include <xpcc/driver/connectivity/usb/USBDevice/USBSerial/USBSerial.h>

#include "pindefs.hpp"
#include "remote_control.hpp"
#include <ch.h>

//#define SWD

using namespace xpcc;
using namespace lpc11;

USB_PRODUCT_STRING("SKYFalcon Radio");
USB_MANUFACTURER_STRING("Ensys.lt");
USB_SERIAL_STRING(GITVER);

Axes axes;
RemoteControl radio;
USBSerial usb;
MAVHandler mavHandler;

StaticIOBuffer<256> txbuf;
StaticIOBuffer<64> rxbuf;
xpcc::lpc11::BufferedUart uart1(115200, txbuf, rxbuf);

__attribute__((section(".fwversion"),used)) const uint32_t fw[5] = {0};

volatile bool boot_detach;

NullIODevice null;
xpcc::log::Logger xpcc::log::debug(uart1);



void idle() {

//	if(!progPin::read()) {
//		NVIC_SystemReset();
//	}

	static PeriodicTimer<> t(1000);
	if(t.isExpired()) {

//		XPCC_LOG_DEBUG .printf("sw rx %d\n", sw_rx.rxAvailable());
//		while(sw_rx.rxAvailable()) {
//			XPCC_LOG_DEBUG << (char)sw_rx.read();
//		}

//		uint32_t t = Timer16B1::getCounterValue();
//		for(int i=0;i<128/8;i++) {
//			//aes128_enc(data+i*16, &ctx);
//
//			xtea_crypt_ecb(&xtea, data+i*8, data+i*8, 0, 0);
//		}
//		XPCC_LOG_DEBUG .printf("enc took %d us\n", Timer16B1::getCounterValue() - t);


		//XPCC_LOG_DEBUG << lpc11::Timer32B0::getCaptureValue() << endl;
		//XPCC_LOG_DEBUG << (uint32_t)LPC_IOCON->PIO0_17 << endl;
		//extern uint32_t __heap_start;

//		XPCC_LOG_DEBUG .printf("freemem %d\n", (uint32_t)&__heap_end - (uint32_t)sbrk(0));
//
		XPCC_LOG_DEBUG .printf("TX %d/%d\n",  radio.getTxGood(), radio.getTxBad());
		XPCC_LOG_DEBUG .printf("RX %d/%d\n",  radio.getRxGood(), radio.getRxBad());
		XPCC_LOG_DEBUG .printf("RSSI %d / Noise %d\n",  radio.getRssi(),radio.getNoiseFloor());


	}

//	int16_t c = uart.read();
//
//	if(c == 's') {
//		XPCC_LOG_DEBUG .printf("Freq %d\n",  radio.getFreq());
//		XPCC_LOG_DEBUG .printf("Cfg# %d\n",  radio.getModemCfg());
//
//	}
//
//	if(c == 's') {
//		uint8_t buffer[128];
//		lpc11u::EEPROM::read((uint8_t*)0, buffer, 128);
//		XPCC_LOG_DEBUG .dump_buffer(buffer, 128);
//
//	}
//	if(c == 'r') {
//		LPC_PMU->GPREG3 &= ~0x01;
//		NVIC_SystemReset();
//	}
//	if(c == 'f') {
//		lpc11u::Watchdog::setTC(0xFFFFFFFF);
//		lpc11u::Watchdog::feed();
//		XPCC_LOG_DEBUG << "f\n";
//		lpc11::IAP::invokeISP();
//	}

	//__WFI();

//	uint8_t c;
//	uart.read(c);
//	if(c=='r') {
//		NVIC_SystemReset();
//	}
//
//	__WFI();
	//lpc11u::Watchdog::feed();
}

void test_osc() {
	//LPC_SYSCON->SYSOSCCTRL |= 1; //enable BYPASS

	//LPC_SYSCON->SYSPLLCLKSEL = 1; //switch to external oscillator

	//LPC_SYSCON->SYSPLLCLKUEN = 1;
	//LPC_SYSCON->SYSPLLCLKUEN = 0;
	//LPC_SYSCON->SYSPLLCLKUEN = 1;
	//while (!(LPC_SYSCON->SYSPLLCLKUEN & 0x01)); //wait until updated

	//LPC_SYSCON->SYSPLLCTRL = 0x25; //input clk 16MHZ, out 48MHZ

	LPC_SYSCON->CLKOUTSEL = 3;
	LPC_SYSCON->CLKOUTUEN =1;
	LPC_SYSCON->CLKOUTUEN =0;
	LPC_SYSCON->CLKOUTUEN =1;
	LPC_SYSCON->CLKOUTDIV = 1;

	lpc11::IOCon::setPinFunc(0, 1, 1);
}

void panic(const char* s) {
	uart_print("PANIC:"); uart_print(s); uart_print("\n");
	while(1);
}

class DFU : public DFUHandler {
	void do_detach() {
		boot_detach = true;
	}
};
DFU dfu;

void usbclk_init() {

	LPC_SYSCON->PDRUNCFG     &= ~(1 << 8); //power up usb pll

	LPC_SYSCON->USBPLLCLKSEL = 1; //switch to external oscillator

	LPC_SYSCON->USBPLLCLKUEN = 1;
	LPC_SYSCON->USBPLLCLKUEN = 0;
	LPC_SYSCON->USBPLLCLKUEN = 1;
	while (!(LPC_SYSCON->USBPLLCLKUEN & 0x01)); //wait until updated

	LPC_SYSCON->USBPLLCTRL = 0x22; //input clk 16MHZ, out 48MHZ

	while (!(LPC_SYSCON->USBPLLSTAT & 0x01)); //wait until PLL locked
}

void bluetooth_init() {

	bt_key::setOutput(1);
	bt_rst::setOutput(0);

	xpcc::sleep(10);
	bt_rst::setOutput(1);

	sw_rx.setBaud(38400);
	Uart1::setBaud(38400);

	IOStream uart(uart1);
	IOStream _u(usb);

	xpcc::sleep(100);

	for(int i = 0; i < 20; i++) {
		uart << "AT\r\n";

		xpcc::sleep(200);

		if(sw_rx.rxAvailable()) {

			uart << "AT+NAME=SKYFalcon_Radio\r\n";
			xpcc::sleep(50);

			uart << "AT+UART=115200,0,0\r\n";
			xpcc::sleep(50);

			char pin[5] = "8888";

			uart << "AT+PSWD=" << pin << "\r\n";
			xpcc::sleep(50);

			while(sw_rx.rxAvailable()) {
				sw_rx.read();
			}

			bt_key::setOutput(0);
			bt_rst::setOutput(0);
			xpcc::sleep(10);
			bt_rst::setOutput(1);

			sw_rx.setBaud(115200);
			Uart1::setBaud(115200);

			break;
		}

	}
}

THD_WORKING_AREA(wa_main_thread, 256);
void main_thread(void*) {
	XPCC_LOG_DEBUG .printf("Starting clock:%d\n", SystemCoreClock);
	XPCC_LOG_DEBUG .printf("Free heap:%d\n", (int)(&__heap_end__)-(int)(&__heap_base__));

#if 0
	bt_rst::setOutput(0);
	bt_key::setOutput(0);
	bluetooth_init();
#endif

	mavHandler.radio = &radio;
	mavHandler.usb = &usb;
	mavHandler.uart = &uart1;
	mavHandler.bluetooth = &sw_rx;

	//GpioInt::attach(0, 13, interrupt, IntEdge::RISING_EDGE);

	//asm("bl 24");
//
//	xpcc::Random::seed();
//
	usb.connect();
	usbConnect::setOutput(true);



	//LPC_PMU->GPREG3 = 0; //tell bootloader: boot OK

	while(1) {
		chThdSleep(MS2ST(1000));

		XPCC_LOG_DEBUG.printf("alive\n");

		if(boot_detach) {
			LPC_PMU->GPREG3 = 255;
			NVIC_SystemReset();
		}

	}
}


THD_TABLE_BEGIN
  THD_TABLE_ENTRY(wa_main_thread, "main", main_thread, NULL)
  THD_TABLE_ENTRY(radio.wa_main_thread, "radio_main", radio.mainTaskEntry, (void*)&radio)
  THD_TABLE_ENTRY(radio.wa_irq_thread, "radio_irq", radio.irqTaskEntry, (void*)&radio)
THD_TABLE_END

extern "C" void port_timer_init();
int main() {

	LPC_WWDT->WARNINT = 1023;
	lpc11u::Watchdog::feed();

	NVIC_EnableIRQ(WDT_IRQn);

	ledRed::setOutput(true);
	ledGreen::setOutput(true);

	usb.addInterfaceHandler(dfu);

	eeprom.initialize();

	NVIC_SetPriority(UART_IRQn, 3);
	NVIC_SetPriority(USB_IRQn, 3);

	NVIC_SetPriority(FLEX_INT0_IRQn, 2); //Radio ISR
	NVIC_SetPriority(TIMER_16_1_IRQn, 2); //PPM decoder
	NVIC_SetPriority(TIMER_32_0_IRQn, 0); //SW uart has max priority

#ifndef SWD
	radioSpiMaster::configurePins(radioSpiMaster::MappingSck::PIO0_10, false);
	radioSpiMaster::initialize(radioSpiMaster::Mode::MODE_0, 100000);
#endif

	usbclk_init();
	port_timer_init();
	chSysInit();

	while(1) {

		lpc11u::Watchdog::feed();

		int16_t c = uart1.read();
		if(c == 'r') {
			NVIC_SystemReset();
		}

	}
}
