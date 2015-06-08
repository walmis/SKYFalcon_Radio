#include <unistd.h>
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/debug.hpp>
#include "eeprom/eeprom.hpp"
#include "SwUart.hpp"
#include "Axes.hpp"
#include "mavHandler.hpp"
#include <xpcc/driver/connectivity/usb/USBDevice.hpp>

#include "pindefs.hpp"
#include "remote_control.hpp"

using namespace xpcc;
using namespace lpc11;

Axes axes;
RemoteControl radio;
USBSerial usb(0xffff, 0x12c9, 0);
MAVHandler mavHandler;
xpcc::lpc11::BufferedUart uart1(115200, 256, 64);

bool boot_detach;

enum { r0, r1, r2, r3, r12, lr, pc, psr};

extern "C" void HardFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}

NullIODevice null;
xpcc::log::Logger xpcc::log::debug(null);

extern "C"
void Hard_Fault_Handler(uint32_t stack[]) {

	//register uint32_t* stack = (uint32_t*)__get_MSP();

//	ledGreen::set(0);
	ledRed::setOutput(0);
	Uart1::init(115200);
	IODeviceWrapper<Uart1> d;
	IOStream w(d);
////
	w.printf("Hard Fault\n");
//
	w.printf("r0  = 0x%08x\n", stack[r0]);
	w.printf("r1  = 0x%08x\n", stack[r1]);
	w.printf("r2  = 0x%08x\n", stack[r2]);
	w.printf("r3  = 0x%08x\n", stack[r3]);
	w.printf("r12 = 0x%08x\n", stack[r12]);
	w.printf("lr  = 0x%08x\n", stack[lr]);
	w.printf("pc  = 0x%08x\n", stack[pc]);
	w.printf("psr = 0x%08x\n", stack[psr]);

	LPC_PMU->GPREG3 |= 1;
	NVIC_SystemReset();
}

//extern "C" void fault_handler() {
//	//ledGreen::set(0);
//	//ledRed::set();
//	dbgPin::setOutput(false);
//	//NVIC_SystemReset();
//	while(1) {
//		if(!progPin::read()) {
//			NVIC_SystemReset();
//		}
//	}
//}

void tick() {
	dbgPin::toggle();
	lpc11u::Watchdog::feed();
	if(!progPin::read()) {
		NVIC_SystemReset();
	}
}

//aes128_ctx_t ctx;
//AVXTEA xtea;

//uint8_t data[256];

void idle() {

	if(!progPin::read()) {
		NVIC_SystemReset();
	}

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
//		XPCC_LOG_DEBUG .printf("TX %d/%d\n",  radio.getTxGood(), radio.getTxBad());
//		XPCC_LOG_DEBUG .printf("RX %d/%d\n",  radio.getRxGood(), radio.getRxBad());
//		XPCC_LOG_DEBUG .printf("RSSI %d / Noise %d\n",  radio.getRssi(),radio.getNoiseFloor());

		if(boot_detach) {
			LPC_PMU->GPREG3 |= 0x01;
			NVIC_SystemReset();
		}
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
	XPCC_LOG_DEBUG << s << endl;
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

int main() {
	LPC_PMU->GPREG3 |= 1;
	lpc11u::Watchdog::init(100000);

	ledRed::setOutput(true);
	ledGreen::setOutput(true);

	usb.addInterfaceHandler(dfu);

	usbclk_init();

	lpc11::SysTickTimer::enable();
	lpc11::SysTickTimer::attachInterrupt(tick);
	//usbConnect::setOutput(false);

	mavHandler.radio = &radio;
	mavHandler.usb = &usb;
	mavHandler.uart1 = &uart1;
	mavHandler.bluetooth = &sw_rx;

	//GpioInt::attach(0, 13, interrupt, IntEdge::RISING_EDGE);

	//asm("bl 24");
//
//	xpcc::Random::seed();
//
	lpc11u::SpiMaster0::configurePins(lpc11u::SpiMaster0::MappingSck::PIO0_10, false);
	lpc11u::SpiMaster0::initialize(lpc11u::SpiMaster0::Mode::MODE_0, 8000000);
//
//	ledRed::setOutput(false);
//	ledGreen::setOutput(false);
//
//	//usbclk_init();
//
	usb.connect();
	usbConnect::setOutput(true);


	eeprom.initialize();

	NVIC_SetPriority(UART_IRQn, 3);
	NVIC_SetPriority(USB_IRQn, 3);

	NVIC_SetPriority(FLEX_INT0_IRQn, 2); //Radio ISR
	NVIC_SetPriority(TIMER_16_1_IRQn, 2); //PPM decoder
	NVIC_SetPriority(TIMER_32_0_IRQn, 0); //SW uart has max priority

	xpcc::TickerTask::tasksRun(idle);
}
