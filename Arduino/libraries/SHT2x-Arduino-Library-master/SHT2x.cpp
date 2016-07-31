/*
  SHT2x - A Humidity Library for Arduino.

  Supported Sensor modules:
    SHT21-Breakout Module - http://www.moderndevice.com/products/sht21-humidity-sensor
	SHT2x-Breakout Module - http://www.misenso.com/products/001
	
  Created by Christopher Ladden at Modern Device on December 2009.
  Modified by Paul Badger March 2010
  
  Modified by www.misenso.com on October 2011:
	- code optimisation
	- compatibility with Arduino 1.0

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>
#include "SHT2x.h"
#include "scmRTOS_Arduino.h"

static const uint16_t POLYNOMIAL = 0x131;

static bool SHT2xCheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
	uint8_t crc = 0;
	uint8_t byteCtr;
	uint8_t bit;
	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	if (crc != checksum)
		return false;
	else
		return true;
}


/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**********************************************************
 * GetHumidity
 *  Gets the current humidity from the sensor.
 *
 * @return float - The relative humidity in %RH
 **********************************************************/
float SHT2xClass::GetHumidity(void)
{
	float data = readSensor(eRHumidityNoHoldCmd);
	if (isnan(data))
		return NAN;
	return (-6.0 + 125.0 * (data / 65536.0));
}

/**********************************************************
 * GetTemperature
 *  Gets the current temperature from the sensor.
 *
 * @return float - The temperature in Deg C
 **********************************************************/
float SHT2xClass::GetTemperature(void)
{
	float data = readSensor(eTempNoHoldCmd);
	if (isnan(data))
		return NAN;
	return (-46.85 + 175.72 * (data / 65536.0));
}

SHT2xClass::SHT2xClass(uint8_t sclPin, uint8_t sdaPin) :
	i2c(sclPin, sdaPin)
{

}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

float SHT2xClass::readSensor(uint8_t command)
{
    float data = NAN;

    do
    {
    	uint8_t r[3];
    	uint16_t tmp;

    	if (!i2c.transfer(eSHT2xAddress | I2C_WRITE, &command, 1))
    		break;

    	OS::sleep(MS_TO_TICKS(100));

    	if (!i2c.transfer(eSHT2xAddress | I2C_READ, r, 3))
    		break;

    	if (!SHT2xCheckCrc(r, 2, r[2]))
    		break;

    	r[1] &= 0xFD;
    	tmp = ((uint16_t)r[0] << 8) | (uint16_t)r[1];

    	if(tmp == 0)
    		break;

    	data = (float)tmp;

    } while(0);

    return data;
}
