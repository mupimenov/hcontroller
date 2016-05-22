#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#include "datetime.h"
#include "cyclogram.h"
#include "program.h"
#include "io.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IOSLOT_START_ADDRESS 0x0000
#define IOSLOT_SIZE 16

#define PROGRAM_START_ADDRESS 0x0400
#define PROGRAM_SIZE 48

/* IO SLOTS */

#define IOSLOTS_COUNT 60

struct common_ioslot /* empty_ioslot */
{
	uint8_t driver;
	uint8_t id;
};

struct analog_input_ioslot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t num;
	uint16_t x1;
	uint16_t x2;
	float y1;
	float y2;
};

struct discrete_input_ioslot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;
	bool inverse;
};

struct discrete_output_ioslot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t operation;
	uint8_t pin;
	bool inverse;
};

enum discrete_output_operation
{
	OPERATION_OR = 0,
	OPERATION_AND
};

struct dhtxx_ioslot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t modification;
	uint8_t parameter;
	uint8_t pin;
};

enum dhtxx_modification
{
	DHTxx_DHT11,
	DHTxx_DHT22
};

enum dhtxx_parameter
{
	DHTxx_TEMPERATURE,
	DHTxx_HUMIDITY
};

struct dallas_temperature_ioslot
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;
};

struct mh_z18_ioslot
{
	uint8_t driver;
	uint8_t id;

	uint8_t receive_pin;
	uint8_t transmit_pin;
};

struct sht2x_ioslot
{
	uint8_t driver;
	uint8_t id;

	uint8_t parameter;

	uint8_t sda_pin;
	uint8_t scl_pin;
};

enum sht2x_parameter
{
	SHT2X_TEMPERATURE,
	SHT2X_HUMIDITY
};

struct abstract_ioslot
{	
	union {
		struct common_ioslot 				common;
		struct analog_input_ioslot 			analog_input;
		struct discrete_input_ioslot 		discrete_input;
		struct discrete_output_ioslot 		discrete_output;
		struct dhtxx_ioslot 				dhtxx;
		struct dallas_temperature_ioslot 	dallas_temperature;
		struct mh_z18_ioslot 				mh_z19;
		struct sht2x_ioslot					sht2x;
	} data;
};

enum ioslot_driver
{
	EMPTY_SLOT_DRIVER = 0,
	ANALOG_INPUT_DRIVER,
	DISCRETE_INPUT_DRIVER,
	DISCRETE_OUTPUT_DRIVER,
	DHTxx_DRIVER,
	DALLAS_TEMPERATURE_DRIVER,
	MH_Z19_DRIVER,
	SHT2X_DRIVER
};

#define ioslot_driver(s) ((s)->data.common.driver)
#define ioslot_id(s) ((s)->data.common.id)

void read_ioslot(uint8_t num, struct abstract_ioslot *ioslot);
void read_ioslot_by_id(uint8_t id, struct abstract_ioslot *ioslot);

/* PROGRAMS */

#define PROGRAMS_COUNT 30

struct common_program /* empty_program */
{
	uint8_t type;
	uint8_t id;
};

struct timer_control_program
{
	uint8_t type;
	uint8_t id;
	
	uint8_t constrains;
	struct datetime from;
	struct datetime to;
	
	struct cyclogram_params cyclogram;
	
	uint8_t output;	
};

struct relay_control_program
{
	uint8_t type;
	uint8_t id;
	
	uint8_t input;
	
	uint8_t constrains;
	struct datetime from;
	struct datetime to;
	
	float low_bound;
	float high_bound;
	
	struct cyclogram_params cyclogram;
	
	uint8_t inverse;
	uint8_t output;	
};

struct pid_control_program
{
	uint8_t type;
	uint8_t id;
	
	uint8_t input;
	
	uint8_t constrains;
	struct datetime from;
	struct datetime to;
	
	float desired;
	
	float proportional_gain;
	float integral_gain;
	float differential_gain;
	
	uint8_t inverse;
	uint8_t output;
};

struct button_control_program
{
	uint8_t type;
	uint8_t id;

	uint8_t input;
	uint8_t output;
};

struct abstract_program
{
	uint8_t (*output)(struct abstract_program *);
	
	union {
		struct common_program 			common;
		struct timer_control_program 	timer;
		struct relay_control_program 	relay;
		struct pid_control_program 		pid;
		struct button_control_program 	button;
	} data;
};

enum program_type
{
	EMPTY_PROGRAM = 0,
	TIMER_CONTROL_PROGRAM,
	RELAY_CONTROL_PROGRAM,
	PID_CONTROL_PROGRAM,
	BUTTON_CONTROL_PROGRAM
};

#define program_type(p) ((p)->data.common.type)
#define program_id(p) ((p)->data.common.id)

void read_program(uint8_t num, struct abstract_program *program);
uint8_t program_count_with_output(uint8_t id);

void config_init(void);

void config_lock(void);
void config_unlock(void);

void config_read(uint16_t address, uint8_t *data, uint16_t size);
void config_write(uint16_t address, const uint8_t *data, uint16_t size);

void config_set_address(uint8_t address);
uint8_t config_get_address(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
