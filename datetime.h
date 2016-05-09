#ifndef DATETIME_H_
#define DATETIME_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct datetime
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t month;
	uint8_t year;
};

enum time_constrains
{
	ALL_TIME = 0,
	STRICT_EQUALITY,
    EVERY_DAY,
    EVERY_MONTH,
    EVERY_YEAR
};

void datetime_set(struct datetime *dt);

struct datetime datetime_now(void);
bool datetime_in(	struct datetime *dt, 
					struct datetime *point1, struct datetime *point2, 
					enum time_constrains constrains);

#ifdef __cplusplus
}
#endif

#endif /* DATETIME_H_ */
