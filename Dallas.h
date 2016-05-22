/*
 * Dallas.h
 *
 *  Created on: May 20, 2016
 *      Author: mupimenov
 */

#ifndef DALLAS_H_
#define DALLAS_H_

#include "OneWire.h"

class Dallas {
public:
	Dallas(uint8_t pin);
	virtual ~Dallas();

	float getTemperature();

private:
	OneWire ds;
};

#endif /* DALLAS_H_ */
