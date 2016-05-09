#include "hydra.h"

#include "link.h"
#include "program_loop.h"

static void led()
{
	OS::sleep(100);
	digitalWrite(13, HIGH);
	OS::sleep(100);
	digitalWrite(13, LOW);
}

scmRTOS_PROCESS(0, 128, led);
scmRTOS_PROCESS(1, 512, link);
scmRTOS_PROCESS(2, 512, prog_loop);

void setup()
{
	pinMode(13, OUTPUT);

	// 1 тик = 10 мс
	TCCR1A = 0;
	TCCR1B = 0;

	// Основание счёта:
	OCR1A = (((F_CPU / 256) * SYSTEM_TIMER_PERIOD_MS) / 1000) - 1;
	// Делитель 256
	TCCR1B = (1 << CS12) | (1 << WGM12);
	TIMSK1 = (1 << OCIE1A);

	link_init();
	program_loop_init();

	scmRTOS_START();
}

void loop()
{
	while (1) {}
}
