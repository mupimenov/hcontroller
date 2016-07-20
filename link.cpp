#include "link.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "Arduino.h"
#include "scmRTOS_Arduino.h"
#include "RTClib.h"

#include "config.h"
#include "io.h"
#include "datetime.h"

#include "modbus.h"
#include "version.h"

#define MODBUS_ADDRESS 0x01

#define LINK_PERIOD_MS 				MS_TO_TICKS(100)

#define LINK_PACKET_SIZE 			((uint8_t)255)
static uint8_t 						recv_buffer[LINK_PACKET_SIZE] = {0};
static uint8_t 						send_buffer[LINK_PACKET_SIZE] = {0};

static struct modbus_instance 		modbus;

static struct adc_input_reg {
	int16_t value[16];
} 									adc_values;


static struct ioslot_value_input_reg {
	float value[60];
} 									ioslot_values;

static const struct modbus_regs_table input_tables[] = {
		{0x0000, sizeof(adc_values) / 2, (uint16_t*)&adc_values},
		{0x1000, sizeof(ioslot_values) / 2, (uint16_t*)&ioslot_values},
		{0, 0, NULL}
};

static struct common_hold_reg {
	struct datetime now;
	uint16_t dummy;
	uint32_t uptime;
	uint16_t modbus_address;
} 									common_values;

static const struct modbus_regs_table holding_tables[] = {
		{0x0000, sizeof(common_values) / 2, (uint16_t*)&common_values},
		{0, 0, NULL}
};

static struct common_coil_bit {
	uint8_t restart_programs;
	uint8_t pause_flag;
} 									controls;

static const struct modbus_bits_table coil_tables[] = {
	{0x0000, sizeof(controls), (uint8_t*)&controls},
	{0, 0, NULL}
};

static int modbus_read(struct modbus_instance *instance, uint8_t *buffer, uint8_t max_size)
{
	int len = 0;
	if (Serial.packetReceived())
	{
		len = Serial.readBytes(buffer, max_size);
		Serial.packetRead();
	}

	return len;
}

static int modbus_write(struct modbus_instance *instance, const uint8_t *packet, uint8_t plen)
{
	Serial.flush();
	Serial.write(packet, plen);

	MODBUS_RETURN(instance, MODBUS_SUCCESS);
}

#define ELEMENT_OFFSET(__table__, __var__) ((uint8_t*)(&(__table__).__var__) - (uint8_t*)(&(__table__)))
#define ELEMENT_IN(__start__, __offset__,__address__, __count__) ((__start__ + __offset__) >= (__address__) && (__start__ + __offset__) <= (__address__ + __count__))

static int modbus_after_write_table(struct modbus_instance *instance, enum modbus_table table, uint16_t address, uint16_t count)
{
	if (table == MODBUS_TABLE_HOLDING_REGISTERS)
	{
		uint16_t offset = ELEMENT_OFFSET(common_values, now);
		if (ELEMENT_IN(0x0000, offset, address * 2, count * 2))
		{
			// SYNC CLOCK
			DateTime dt(common_values.now.year, common_values.now.month, common_values.now.day,
					common_values.now.hours, common_values.now.minutes, common_values.now.seconds);

			OS::TSysTimerLocker lock;
			RTC_DS1307 rtc;
			rtc.begin();
			rtc.adjust(dt);
		}

		offset = ELEMENT_OFFSET(common_values, modbus_address);
		if (ELEMENT_IN(0x0000, offset, address * 2, count * 2))
		{
			config_lock();
			config_set_address(common_values.modbus_address & 0xFF);
			config_unlock();

			// instance->address = common_values.modbus_address;
		}
	}
	
	if (table == MODBUS_TABLE_COILS)
	{
		uint16_t offset = ELEMENT_OFFSET(controls, restart_programs);
		if (ELEMENT_IN(0x0000, offset, address, count))
		{
			if (controls.restart_programs == 0x01)
			{
				controls.restart_programs = 0x00;
				
				program_reset();
			}
		}

		offset = ELEMENT_OFFSET(controls, pause_flag);
		if (ELEMENT_IN(0x0000, offset, address, count))
		{
			if (controls.pause_flag == 0x01)
			{
				config_set_pause_flag(controls.pause_flag);
			}
		}
	}

	MODBUS_RETURN(instance, MODBUS_SUCCESS);
}

static int modbus_read_file(struct modbus_instance *instance, uint16_t filenum, uint16_t address, uint16_t count, uint8_t *data)
{
	uint16_t start_address;
	bool found = false;

	if (filenum == 0x0001)
	{
		start_address = IOSLOT_START_ADDRESS;
		found = true;
	}
	else if (filenum == 0x0002)
	{
		start_address = PROGRAM_START_ADDRESS;
		found = true;
	}

	if (found)
	{
		uint16_t i;
		uint16_t j;

		for (i = 0, j = 0; i < count; ++i, ++address, j += 2)
		{
			uint8_t r[2];
			config_read(start_address + (address << 1), r, 2);

			data[j] = r[1];
			data[j + 1] = r[0];
		}

		MODBUS_RETURN(instance, MODBUS_SUCCESS);
	}

	MODBUS_RETURN(instance, MODBUS_BAD_PARAMS);
}

static int modbus_write_file(struct modbus_instance *instance, uint16_t filenum, uint16_t address, uint16_t count, const uint8_t *data)
{
	uint16_t start_address;
	bool found = false;

	if (filenum == 0x0001)
	{
		start_address = IOSLOT_START_ADDRESS;
		found = true;
	}
	else if (filenum == 0x0002)
	{
		start_address = PROGRAM_START_ADDRESS;
		found = true;
	}

	if (found)
	{
		uint16_t i;
		uint16_t j;

		config_lock();

		for (i = 0, j = 0; i < count; ++i, ++address, j += 2)
		{
			uint8_t w[2];

			w[0] = data[j + 1];
			w[1] = data[j];
			config_write(start_address + (address << 1), w, 2);
		}

		config_unlock();
		
		if (filenum == 0x0001)
		{
			io_reset();
		}
		else if (filenum == 0x0002)
		{
			program_reset();
		}

		MODBUS_RETURN(instance, MODBUS_SUCCESS);
	}

	MODBUS_RETURN(instance, MODBUS_BAD_PARAMS);
}

static const struct modbus_functions modbus_fn = {
		/*.open*/ 				NULL,
		/*.write*/				modbus_write,
		/*.read*/				modbus_read,
		/*.close*/				NULL,
		/*.lock*/				NULL,
		/*.before_read_table*/	NULL,
		/*.after_write_table*/	modbus_after_write_table,
		/*.read_file*/			modbus_read_file,
		/*.write_file*/			modbus_write_file
};

static void update_inputs(void)
{
	uint8_t i;

	io_lock();

	for (i = 0; i < 16; ++i)
	{
		adc_values.value[i] = get_adc_value(i);
	}

	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		ioslot_values.value[i] = get_ioslot_value(i);
	}

	io_unlock();

	common_values.now = datetime_now();
	common_values.uptime = millis();
	common_values.modbus_address = config_get_address();

	controls.pause_flag = config_get_pause_flag();
}

void link(void)
{
	int ret = modbus_io(&modbus);
	if (ret == MODBUS_SUCCESS)
	{
		// OK
	}

	update_inputs();

	OS::sleep(LINK_PERIOD_MS);
}

void link_init(void)
{
	uint8_t address;

	Serial.begin(9600);
	Serial.setTimeout(0);
	Serial.setPacketTimeout(10);

	modbus.arg = NULL;

	address = config_get_address();
	if (address < 1 || address > 247)
		address = MODBUS_ADDRESS;

	modbus.address = address;
	modbus.recv_buffer = recv_buffer;
	modbus.recv_buffer_size = LINK_PACKET_SIZE;
	modbus.send_buffer = send_buffer;
	modbus.send_buffer_size = LINK_PACKET_SIZE;

	modbus.id = (uint8_t*)("HPONIC-" VERSION);

	modbus.coil_tables = coil_tables;
	modbus.discrete_tables = NULL;
	modbus.input_tables = input_tables;
	modbus.holding_tables = holding_tables;

	modbus.functions = &modbus_fn;
}

