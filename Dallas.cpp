/*
 * Dallas.cpp
 *
 *  Created on: May 20, 2016
 *      Author: mupimenov
 */

#include "Dallas.h"

Dallas::Dallas(uint8_t pin) :
	ds(pin)
{
}

Dallas::~Dallas() {
}

float Dallas::getTemperature() {
	float temperature = NAN;
	uint8_t data[9];
	int i;

	ds.reset();
	ds.write(0x33);
	for (i = 0; i < 8; ++i)
		data[i] = ds.read();

	if (OneWire::crc8(data, 7) == data[7])
	{
		uint8_t family_code = data[0];
		uint8_t type_s;

		switch (family_code) {
		case 0x10:
			type_s = 1; // Chip = DS18S20
			break;
		case 0x28:
			type_s = 0; // Chip = DS18B20
			break;
		case 0x22:
			type_s = 0; // Chip = DS1822
			break;
		default:
			return temperature;
		}

		ds.reset();
		ds.write(0xCC);
		ds.write(0xBE);
		for (i = 0; i < 9; ++i)
			data[i] = ds.read();

		if (OneWire::crc8(data, 8) == data[8])
		{
			int16_t raw = (data[1] << 8) | data[0];
			if (type_s) {
				raw = raw << 3; // 9 bit resolution default
				if (data[7] == 0x10) {
					// "count remain" gives full 12 bit resolution
					raw = (raw & 0xFFF0) + 12 - data[6];
				}
			} else {
				byte cfg = (data[4] & 0x60);
				// at lower res, the low bits are undefined, so let's zero them
				if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
				else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
				else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
				//// default is 12 bit resolution, 750 ms conversion time
			}

			temperature = (float)raw / 16.0;
		}


	}

	ds.reset();
	ds.write(0xCC);
	ds.write(0x44);

	return temperature;
}
