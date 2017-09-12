#include <xpcc/processing/rtos_abstraction.hpp>
#include <ch.h>
 #include <chbsem.h>
#include <xpcc/architecture.hpp>

namespace xpcc {

struct Data {
	binary_semaphore_t sem;
	thread_t* owner;
	int count;
};

RMutex::RMutex() {
	data = new Data;
	Data* d = (Data*)data;
	d->owner = 0;
	d->count = 0;
	chBSemObjectInit(&d->sem, 0);
}

RMutex::~RMutex() {
	delete (Data*)data;
}

void RMutex::lock() {
	lock(TIME_INFINITE);
}

bool RMutex::lock(uint32_t timeout) {

	Data* d = (Data*)data;
	msg_t status;

	chSysLock();

	if(chThdGetSelfX() == d->owner) {
		d->count++;
	} else {
		status = chBSemWaitTimeoutS(&d->sem, timeout);
		if(status == MSG_TIMEOUT) {
			chSysUnlock();
			return false;
		}
		d->owner = chThdGetSelfX();
		d->count = 1;
	}
	chSysUnlock();

	return true;
}

bool RMutex::try_lock() {
	return lock(TIME_IMMEDIATE);
}

void RMutex::unlock() {
	chSysLock();
	Data* d = (Data*)data;
	if(chThdGetSelfX() == d->owner) {
		d->count--;
		//XPCC_LOG_DEBUG.printf("unlock %x %d\n", d->owner, d->count);
		if(!d->count) {
			//XPCC_LOG_DEBUG.printf("mutex release %x\n", d->owner);
			d->owner = 0;
			chBSemSignalI(&d->sem);
			chSchRescheduleS();
		}
	}
	chSysUnlock();
}

}
