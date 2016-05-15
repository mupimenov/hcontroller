/*
 * MH_Z19.h
 */

#ifndef MH_Z19_H_
#define MH_Z19_H_

#include <SoftwareSerial.h>

class MH_Z19 {
public:
	MH_Z19(uint8_t receivePin, uint8_t transmitPin);
	virtual ~MH_Z19();

	void begin();
	float read();

protected:
	uint8_t computeCrc(const uint8_t *data, size_t size);
private:
	SoftwareSerial serial;
};

#endif /* MH_Z19_H_ */
