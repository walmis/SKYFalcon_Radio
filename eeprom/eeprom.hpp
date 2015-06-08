/*
 *
 *  Created on: May 7, 2014
 *      Author: walmis
 */
#pragma once

#include <xpcc/architecture.hpp>

#include "eedata.hpp"

extern void panic(const char* s);

class Eeprom  {
public:
	Eeprom() : token(0) {}
	virtual ~Eeprom(){};

	template<typename T, typename U> constexpr size_t offsetOf(U T::*member)
	{
	    return (char*)&((T*)nullptr->*member) - (char*)nullptr;
	}

	void initialize()  {
		if(!xpcc::lpc11u::EEPROM::read((uint8_t*)0, &token, 1)) {
			panic("eeprom init failed");
		}

		if(token != TOKEN) {
			write(0, (uint8_t*)&eeDefaults, sizeof(EEData));
		}
	}

	template <typename T, typename U, typename Y>
	bool put(T U::*pos, Y &data) {
		static_assert(sizeof(T) == sizeof(Y), "Type size mismatch");
		return write(offsetOf(pos), (uint8_t*)&data, sizeof(T));
	}

	template <typename T, typename U, typename Y>
	bool get(T U::*pos, Y &dest) {
		static_assert(sizeof(T) <= sizeof(Y), "Type size mismatch");
		return read(offsetOf(pos), (uint8_t*)&dest, sizeof(T));
	}

	bool read(uint32_t ofs, uint8_t* addr, size_t len) {
		return xpcc::lpc11u::EEPROM::read((uint8_t*)ofs, addr, len);
	}

	bool write(uint32_t ofs, uint8_t* addr, size_t len) {
		if(addr < (uint8_t*)0x10000000) { //write from flash not supported
			uint8_t tmp[len];
			memcpy(tmp, addr, len);
			return xpcc::lpc11u::EEPROM::write((uint8_t*)ofs, tmp, len);
		}
		return xpcc::lpc11u::EEPROM::write((uint8_t*)ofs, addr, len);
	}

	bool isValidToken() {
		return token == TOKEN;
	}

	void setToken() {
		uint8_t t = TOKEN;
		write(0, &t, 1);
		token = TOKEN;
	}

protected:
	uint8_t token;
};

extern Eeprom eeprom;

