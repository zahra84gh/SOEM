#include <stdint.h>

// MODES
#define PAN355_OPMODE_IDLE 0
#define PAN355_OPMODE_HOLD 1
#define PAN355_OPMODE_POSITION 2
#define PAN355_OPMODE_VELOCITY 3
#define PAN355_OPMODE_DO_TORQUE_HOME 100
#define PAN355_OPMODE_HOME_TP_CURRENT_ENC_POSITION 101
#define PAN355_OPMODE_SET_HOME 102
#define PAN355_OPMODE_RAW_ENCODER_POSIRION 200

// SDO SETTIND
#define PDO_COM_TIMEOUT_CONFIGURATION 0x3005
#define STP0_CONFIGURATION 0x3010
#define STP1_CONFIGURATION 0x3020
#define STP0_IN 0x6010
#define STP1_IN 0x6020
#define STP0_OUT 0x7010
#define STP1_OUT 0x7020

// PARAM VALUE
#define timeout_value 1000
#define stp_0_action_value 1
#define stp_1_action_value 1
#define pulse_div_value 0
#define ramp_div_value 8
#define i_hold_value 10
#define i_run_value 15
#define max_pos_error_value 0
#define current_timeout_value 0
#define microstep_setting_value 8
#define invert_motor_dir_value 0

void set_timeout(uint16_t timeout);
void set_stp_0_action(uint8_t stp_0_action);
void set_stp_1_action(uint8_t stp_1_action);
void set_stp_pulse_div(uint16_t STP_CONFIGURATION, uint8_t pulse_div);
void set_stp_ramp_div(uint16_t STP_CONFIGURATION, uint8_t ramp_div);
void set_stp_i_hold(uint16_t STP_CONFIGURATION, uint8_t i_hold);
void set_stp_i_run(uint16_t STP_CONFIGURATION, uint8_t i_run);
void set_stp_max_position_error(uint16_t STP_CONFIGURATION, uint32_t max_pos_error);
void set_stp_run_current_timeout(uint16_t STP_CONFIGURATION, uint32_t current_timeout);
void set_stp_microstep_setting(uint16_t STP_CONFIGURATION, uint8_t microstep_setting);
void set_stp_invert_motor_direction(uint16_t STP_CONFIGURATION, uint8_t invert_motor_dir);

typedef struct __attribute__((__packed__))
{
   uint8_t stp_requested_opmode_0;
   uint8_t padding_0;
   uint16_t stp_target_acceleration_0;
   uint16_t stp_target_velocity_0;
   uint32_t stp_target_position_0;
   uint8_t stp_requested_opmode_1;
   uint8_t padding_1;
   uint16_t stp_target_acceleration_1;
   uint16_t stp_target_velocity_1;
   uint32_t stp_target_position_1;
} ec_slave_stp_output;

typedef struct __attribute__((__packed__))
{
   uint8_t status_0;
   uint8_t error_0;
   uint8_t stp_actual_opmode_0;
   uint8_t stp_error_0;
   uint8_t stp_status_0;
   uint8_t padding_0;
   uint16_t stp_actual_acceleration_0;
   uint16_t stp_actual_velocity_0;
   uint32_t stp_actual_position_0;
   uint32_t stp_actual_position_error_0;
   uint8_t stp_actual_opmode_1;
   uint8_t stp_error_1;
   uint8_t stp_status_1;
   uint8_t padding_1;
   uint16_t stp_actual_acceleration_1;
   uint16_t stp_actual_velocity_1;
   uint32_t stp_actual_position_1;
   uint32_t stp_actual_position_error_1;
   uint16_t adc_status;
   uint32_t adc_value;
} ec_slave_stp_input;
