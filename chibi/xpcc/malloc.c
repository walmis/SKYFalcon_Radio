/*
 * malloc.c
 *
 *  Created on: Sep 1, 2017
 *      Author: walmis
 */

#include <ch.h>
#include <chheap.h>

__attribute((used)) void __user_late_init()  {
	_core_init();
	_heap_init();
}

void* malloc(size_t size) {
	return chHeapAlloc(0, size);
}

void free(void* ptr) {
	chHeapFree(ptr);
}


