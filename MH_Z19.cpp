/*
 * MH_Z19.cpp
 */

#include "MH_Z19.h"

#include <math.h>

MH_Z19::MH_Z19(uint8_t receivePin, uint8_t transmitPin) :
	serial(receivePin, transmitPin)
{

}

MH_Z19::~MH_Z19() {
}

void MH_Z19::begin() {
	serial.begin(9600);
}

float MH_Z19::read() {
	const uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
	uint8_t response[9];
	serial.write(cmd, 9);
	serial.readBytes(response, 9);
	uint8_t crc = computeCrc(response, 9);
	if (!(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc))
		return NAN;

	return (float)(((uint16_t)response[2] << 8) | (uint16_t)response[3]);
}

uint8_t MH_Z19::computeCrc(const uint8_t* response, size_t size) {
	uint8_t crc = 0;
	for (size_t i = 1; i < size - 1; i++)
		crc+=response[i];
	crc = 0xFF - crc;
	crc++;
	return crc;
}
