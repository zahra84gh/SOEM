#ifndef IO116E_H
#define IO116E_H

#include <stdint.h>


typedef struct __attribute__((__packed__)) {
	uint8_t status;
	uint8_t error;
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
} io116e_input_map_t;


typedef struct __attribute__((__packed__)) {
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
	uint8_t pulse_offset_0;
	uint8_t pulse_offset_1;
	uint32_t pulse_count_0;
	uint32_t pulse_count_1;
	uint32_t pulse_count_2;
	uint32_t pulse_count_3;
	uint8_t pulse_updated;
	uint8_t padding2;
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
	uint32_t trigger_enabled;
	uint32_t trigger_updated;
	uint16_t in_count_command;
	uint8_t in_count_latch_command;
	uint8_t in_count_latch_updated;
} io116e_output_map_t;

enum input_states {
	OFF = 0,
	RISING = 1,
	ON = 2,
	FALLING = 3
};


void io116e_init(uint8_t*, uint8_t*, int,
				 uint8_t*, uint8_t*, int,
				 uint8_t*, uint8_t*, int);
void io116e_set_pdo_delay_behaviour(void);
int io116e_get_emergency_state(void);
int io116e_get_reset_button_state(void);
void io116e_set_enclosure_leds(uint16_t);
void io116e_start_end_welding_laser(void);
void io116e_stop_end_welding_laser(void);
void io116e_start_handle_laser(void);
void io116e_stop_handle_laser(void);
enum input_states handle_start_button(void);
enum input_states handle_g_stop_os(void);
enum input_states handle_g_stop_nos(void);
	
int io116e_get_laser_ready_state(void);

void io116e_right_finger_up(void);
void io116e_right_finger_down(void);
void io116e_left_finger_up(void);
void io116e_left_finger_down(void);

int io166e_get_infeed_photocell(void);
int io116e_get_start_button_state(void);

int io116e_get_infeed_verification_photocell(void);
int io116e_get_infeed_sync_photocell(void);
int io116e_get_infeed_mid_photocell(void);
int io116e_get_gate_photocell_state(void);

void io116e_tube_welder_on(void);
void io116e_tube_welder_off(void);
void io116e_end_welder_on(void);
void io116e_end_welder_off(void);

int io116e_get_end_welder_jaw_left_sensor_state(void);
int io116e_get_end_welder_jaw_right_sensor_state(void);

int io116e_get_finger_sync_photocell(void);

void io116e_set_print_trigger(void);
void io116e_remove_print_trigger(void);

#endif