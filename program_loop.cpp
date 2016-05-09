#include "program_loop.h"

#include "scmRTOS_Arduino.h"
#include "RTClib.h"

#include "config.h"
#include "io.h"
#include "program.h"

#define PROG_LOOP_PERIOD_MS 	MS_TO_TICKS(100)

static void update_common_values(void)
{
	RTC_DS1307 rtc;
	rtc.begin();
	DateTime dt = rtc.now();

	struct datetime dt2;
	dt2.hours = dt.hour();
	dt2.minutes = dt.minute();
	dt2.seconds = dt.second();
	dt2.day = dt.day();
	dt2.month = dt.month();
	dt2.year = dt.year() - 2000;

	datetime_set(&dt2);
}

void prog_loop(void)
{
	update_common_values();

	io_execute_in();
	program_execute();
	io_execute_out();

	OS::sleep(PROG_LOOP_PERIOD_MS);
}

void program_loop_init(void)
{
	config_init();
	io_init();
	program_init();
}
