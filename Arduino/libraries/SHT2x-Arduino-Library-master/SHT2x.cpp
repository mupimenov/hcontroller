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
	float data = readSensor(eRHumidityHoldCmd);
	if (isnan(data))
		return NAN;
	return (-6.0 + 125.0 / 65536.0 * data);
}

/**********************************************************
 * GetTemperature
 *  Gets the current temperature from the sensor.
 *
 * @return float - The temperature in Deg C
 **********************************************************/
float SHT2xClass::GetTemperature(void)
{
	float data = readSensor(eTempHoldCmd);
	if (isnan(data))
		return NAN;
	return (-46.85 + 175.72 / 65536.0 * data);
}

SHT2xClass::SHT2xClass(uint8_t sdaPin, uint8_t sclPin) :
	i2c(sdaPin, sclPin)
{

}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

float SHT2xClass::readSensor(uint8_t command)
{
    float data = NAN;

    if (i2c.start(eSHT2xAddress))
    {
    	i2c.write(command);
    	OS::sleep(MS_TO_TICKS(10));
    	i2c.stop();

    	OS::sleep(MS_TO_TICKS(100));

    	if (i2c.start(eSHT2xAddress | 1))
    	{
    		int16_t tmp = ((i2c.read(0)) << 8);
    		tmp += i2c.read(1);
    		tmp &= ~0x0003;

    		data = (float)tmp;
    	}
    }

    return data;
}
