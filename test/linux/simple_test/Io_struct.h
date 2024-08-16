
#include <stdint.h>


typedef struct __attribute__((__packed__))
{
	uint16_t output_cmd;
	uint8_t out_prescale_0;
	uint8_t out_prescale_1;
	uint8_t out_prescale_2;
	uint8_t out_prescale_3;
	uint8_t out_update;
	uint8_t padding0;
	uint16_t in_prescale_0;
	uint16_t in_prescale_1;
	uint16_t in_prescale_2;
	uint16_t in_prescale_3;
	uint16_t in_prescale_4;
	uint16_t in_prescale_5;
	uint16_t in_prescale_6;
	uint16_t in_prescale_7;
	uint16_t in_update;
	uint32_t in_filter;
	uint8_t in_filter_enabled;
	uint8_t padding1;
	uint8_t duty_cycle_0;
	uint8_t duty_cycle_1;
	uint8_t duty_cycle_2;
	uint8_t duty_cycle_3;
	uint8_t offset_0;
	uint8_t offset_1;
	uint32_t pulse_count_0;
	uint32_t pulse_count_1;
	uint32_t pulse_count_2;
	uint32_t pulse_count_3;
	uint8_t pulse_updated;
	uint8_t padding2;
    uint16_t enable;
	uint32_t trigger_count_0;
	uint32_t trigger_count_1;
	uint32_t trigger_count_2;
	uint32_t trigger_count_3;
	uint32_t trigger_count_4;
	uint32_t trigger_count_5;
	uint32_t trigger_count_6;
	uint32_t trigger_count_7;
	uint32_t trigger_count_8;
	uint32_t trigger_count_9;
	uint32_t trigger_count_10;
	uint32_t trigger_count_11;
	uint32_t trigger_count_12;
	uint32_t trigger_count_13;
	uint32_t trigger_count_14;
	uint32_t trigger_count_15;
	uint32_t trigger_updated;
	uint16_t in_count_command;
	uint8_t in_count_latch_command;
	uint8_t in_count_latch_updated;
    uint8_t leds;
    uint8_t padding3;
}ec_slave_1_output;


 typedef struct __attribute__((__packed__))
 {
	uint16_t status;
	uint16_t error;
	uint16_t temperature;
	uint16_t output_state;
	uint8_t input_state;
	uint8_t padding;
	uint16_t triggered;
	uint32_t count_0;
	uint32_t count_1;
	uint32_t count_2;
	uint32_t count_3;
	uint32_t count_4;
	uint32_t count_5;
	uint32_t count_6;
	uint32_t count_7;
	uint32_t adv_count_0;
	uint32_t adv_count_1;
	uint32_t adv_count_2;
	uint32_t adv_count_3;
	uint32_t adv_count_4;
	uint32_t adv_count_5;
	uint32_t adv_count_6;
	uint32_t adv_count_7;
}ec_slave_1_input;




