/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Io_struct.h"
#include "step_motor.h"
#include "ethercat.h"
#include "mx2_inverter.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

void print_binary(unsigned int value, int bits)
{
   for (int i = bits - 1; i >= 0; i--)
   {
      printf("%d", (value >> i) & 1);
      if (i % 4 == 0 && i != 0)
      {
         printf(" ");
      }
   }
   printf("\n");
}

void set_timeout(uint16_t timeout)
{
   ec_SDOwrite(1, PDO_COM_TIMEOUT_CONFIGURATION, 0x01, FALSE, sizeof(timeout), &timeout, EC_TIMEOUTRXM);
}

void set_stp_0_action(uint8_t stp_0_action)
{
   ec_SDOwrite(1, PDO_COM_TIMEOUT_CONFIGURATION, 0x02, FALSE, sizeof(stp_0_action), &stp_0_action, EC_TIMEOUTRXM);
}

void set_stp_1_action(uint8_t stp_1_action)
{
   ec_SDOwrite(1, PDO_COM_TIMEOUT_CONFIGURATION, 0x03, FALSE, sizeof(stp_1_action), &stp_1_action, EC_TIMEOUTRXM);
}

void set_stp_pulse_div(uint16_t STP_CONFIGURATION, uint8_t pulse_div)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x01, FALSE, sizeof(pulse_div), &pulse_div, EC_TIMEOUTRXM);
}

void set_stp_ramp_div(uint16_t STP_CONFIGURATION, uint8_t ramp_div)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x02, FALSE, sizeof(ramp_div), &ramp_div, EC_TIMEOUTRXM);
}

void set_stp_i_hold(uint16_t STP_CONFIGURATION, uint8_t i_hold)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x03, FALSE, sizeof(i_hold), &i_hold, EC_TIMEOUTRXM);
}

void set_stp_i_run(uint16_t STP_CONFIGURATION, uint8_t i_run)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x04, FALSE, sizeof(i_run), &i_run, EC_TIMEOUTRXM);
}

void set_stp_max_position_error(uint16_t STP_CONFIGURATION, uint32_t max_pos_error)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x05, FALSE, sizeof(max_pos_error), &max_pos_error, EC_TIMEOUTRXM);
}

void set_stp_run_current_timeout(uint16_t STP_CONFIGURATION, uint32_t current_timeout)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x06, FALSE, sizeof(current_timeout), &current_timeout, EC_TIMEOUTRXM);
}

void set_stp_microstep_setting(uint16_t STP_CONFIGURATION, uint8_t microstep_setting)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x07, FALSE, sizeof(microstep_setting), &microstep_setting, EC_TIMEOUTRXM);
}

void set_stp_invert_motor_direction(uint16_t STP_CONFIGURATION, uint8_t invert_motor_dir)
{
   ec_SDOwrite(1, STP_CONFIGURATION, 0x08, FALSE, sizeof(invert_motor_dir), &invert_motor_dir, EC_TIMEOUTRXM);
}

static int initialize_ethercat(char *ifname)
{
   int i;
   inOP = FALSE;
   int IO_map_size;
   int device_state;

   printf("ec: initialize_ethercat: start\n");

   /* Configure Ethercat master and bind socket to ifname */
   if (!ec_init(ifname))
   {
      printf("ec: initialize_ethercat: Failed to use %s - abort", ifname);
      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: ec_init on %s succeeded.\n", ifname);

   /* Enumerate and init all slaves */
   if (ec_config_init(FALSE) <= 0)
   {
      printf("ec: initialize_ethercat: Failed to init slaves - abort\n");
      ec_close();
      return -1;
   }

   /* Set slaves in init state */
   ec_slave[0].state = EC_STATE_INIT + EC_STATE_ACK;
   ec_writestate(0);

   device_state = ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE * 1);

   if (device_state != EC_STATE_INIT)
   {
      printf("ec: initialize_ethercat: Unable to set device in boot state device_state=%d - abort\n", device_state);
      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: init state reached\n");

   /* Set slaves in boot state and clear all errors (if any) */
   ec_slave[0].state = EC_STATE_BOOT + EC_STATE_ACK;
   ec_writestate(0);

   /* For some reason is this ready call needed */
   ec_readstate();

   device_state = ec_statecheck(0, EC_STATE_BOOT, EC_TIMEOUTSTATE * 1);

   if (device_state != EC_STATE_BOOT && device_state != EC_STATE_INIT)
   {
      printf("ec: initialize_ethercat: Unable to set device in boot or init state device_state=%d - abort\n", device_state);
      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: boot/init state reached - device_state=%d\n", device_state);

   /* Set slaves in init state */
   ec_slave[0].state = EC_STATE_INIT + EC_STATE_ACK;
   ec_writestate(0);

   device_state = ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE * 1);

   if (device_state != EC_STATE_INIT)
   {
      printf("ec: initialize_ethercat: Unable to set device in init state device_state=%d - abort\n", device_state);
      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: init state reached\n");

   /* Wait for slaves to come up */
   int temp_count = 0;
   do
   {
      ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE * 4);
      uint16_t w;
      temp_count = ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);
   } while (temp_count < ec_slavecount);

   printf("ec: initialize_ethercat: All slaves are back after init/boot/init seq.\n");

   /* Enumerate and init all slaves again after we have left boot state */
   if (ec_config_init(FALSE) <= 0)
   {
      printf("ec: initialize_ethercat: Failed to config init slaves - abort\n");
      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: ec_config_init again after boot state\n");

   /* Set slaves in pre-op state */
   ec_slave[0].state = EC_STATE_PRE_OP;
   ec_writestate(0);

   device_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 1);

   if (device_state != EC_STATE_PRE_OP)
   {
      printf("ec: initialize_ethercat: Unable to set device in pre-op state device_state=%d - abort\n", device_state);

      for (int i = 1; i <= ec_slavecount; i++)
      {
         device_state = ec_statecheck(i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 1);
         printf("\t slave %d state %d\n", i, device_state);
      }

      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: pre-op state reached\n");

   /* Map all PDOs from slaves to IOmap with Outputs/Inputs in sequential order (legacy SOEM way).*/
   IO_map_size = ec_config_map(&IOmap);

   printf("\tec: initialize_ethercat: IO_map_size %d\n", IO_map_size);

   /* Check actual slave state. This is a blocking function. */
   device_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

   if (device_state != EC_STATE_SAFE_OP)
   {
      printf("ec: initialize_ethercat: Unable to set device in safe-op state device_state=%d - abort\n", device_state);
      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: save-op state reached\n");

   /* Make sure all slaves are in safe-op */
   if (ec_slave[0].state != EC_STATE_SAFE_OP)
   {
      printf("ec: initialize_ethercat: Not all slaves reached safe operational state device_state= %d - abort\n", device_state);
      ec_readstate();
      for (i = 1; i <= ec_slavecount; i++)
      {
         if (ec_slave[i].state != EC_STATE_SAFE_OP)
         {
            printf("ec: initialize_ethercat: Slave %d State=%2x StatusCode=%4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
         }
      }

      ec_close();
      return -1;
   }

   printf("ec: initialize_ethercat: initialized\n");

   return 0;
}

static int ethercat_loop_counter = 0;
static void ethercat_loop(void)
{
   int i, chk;
   int slave_state;
   inOP = FALSE;
   // ec_slave_1_output *out_ptr;
   // ec_slave_1_input *in_ptr;
   // unsigned int bit_to_set = 0;
   // ec_slave_stp_output *stp_out_ptr;
   // ec_slave_stp_input *stp_in_ptr;

   printf("ec: ethercat_loop: start\n");

   /* Ethercat house keeping to get it going in operational state */
   expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
   printf("ec: ethercat_loop: Calculated workcounter %d\n", expectedWKC);
   ec_slave[0].state = EC_STATE_OPERATIONAL;
   /* send one valid process data to make outputs in slaves happy */
   ec_send_processdata();
   ec_receive_processdata(EC_TIMEOUTRET);
   /* request OP state for all slaves */
   ec_writestate(0);

   /* Initialize number of checks before giving up */
   chk = 40;

   /* wait for all slaves to reach operational state */
   do
   {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      slave_state = ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
   } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

   printf("ec: ethercat_loop: All slaves should be in operational state : new state = %d chk=%d\n", slave_state, chk);

   if (ec_slave[0].state != EC_STATE_OPERATIONAL)
   {
      printf("ec: ethercat_loop: Not all slaves reached operational state.\n");
      ec_readstate();

      for (i = 1; i <= ec_slavecount; i++)
      {
         if (ec_slave[i].state != EC_STATE_OPERATIONAL)
            printf("ec: ethercat_loop: Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }

      printf("ec: ethercat_loop: Unable to get all ethercat devices in operation mode - giving up :(\n");
      return;
   }

   inOP = TRUE;

   ethercat_loop_counter = 0;

   printf("Start ethercat loop\n");

   // out_ptr = (ec_slave_1_output *)ec_slave[2].outputs;
   // in_ptr = (ec_slave_1_input *)ec_slave[2].inputs;
   //------------------------------------------------------------------------

   // SDO SETTING
   // stp_out_ptr = (ec_slave_stp_output *)ec_slave[1].outputs;
   // stp_in_ptr = (ec_slave_stp_input *)ec_slave[1].inputs;
   /*  set_timeout(1000);
     set_stp_0_action(1);
     set_stp_1_action(1);
     set_stp_pulse_div(STP0_CONFIGURATION, pulse_div_value);
     set_stp_ramp_div(STP0_CONFIGURATION, ramp_div_value);
     set_stp_i_hold(STP0_CONFIGURATION, i_hold_value);
     set_stp_i_run(STP0_CONFIGURATION, i_run_value);
     set_stp_max_position_error(STP0_CONFIGURATION, max_pos_error_value);
     set_stp_run_current_timeout(STP0_CONFIGURATION, current_timeout_value);
     set_stp_microstep_setting(STP0_CONFIGURATION, microstep_setting_value);
     set_stp_invert_motor_direction(STP0_CONFIGURATION, invert_motor_dir_value);

     uint32_t initial_position = 0;
     uint32_t target_position = 0;
     boolean moving_to_position = FALSE;
   */
   //...............................................................................................................
   mx2_output* mx2_out_ptr = (mx2_output*)ec_slave[1].outputs;
   mx2_input* mx2_in_ptr = (mx2_input*)ec_slave[1].inputs;

   /* Ethercat cyclic loop */
   while (1)
   {
      /*
       if (ethercat_loop_counter == 10)
       {
          stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_VELOCITY;
          stp_out_ptr->stp_target_acceleration_0 = 1000;
          stp_out_ptr->stp_target_velocity_0 = 100;
       }
       */

      ethercat_loop_counter++;

      if (ethercat_loop_counter % 200 == 10)
         printf("ethercat_loop_counter %d slaves %d\n", ethercat_loop_counter, ec_slavecount);

      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);

      /* If work counter is not as expected - report and continue */
      if (wkc < expectedWKC)
      {
         printf("wkc not increasing wkc=%d expectedWKC=%d\n", wkc, expectedWKC);
         osal_usleep(5000);
         continue;
      }

      /*   if (ethercat_loop_counter % 200 == 0)
        {
           printf("step motor actual position:  %d\n"
                  "opmode: %d\n"
                  "velocity: %d\n"
                  "acceleration: %d\n",
                  stp_in_ptr->stp_actual_position_0,
                  stp_out_ptr->stp_requested_opmode_0,
                  stp_out_ptr->stp_target_velocity_0,
                  stp_out_ptr->stp_target_acceleration_0);

           //   bit_to_set = (ethercat_loop_counter / 200) - 1;
              if (bit_to_set <= 15)
             {
                out_ptr->output_cmd = 0;
                out_ptr->output_cmd |= (1 << bit_to_set);
                print_binary(out_ptr->output_cmd, 16);
             }
        } */

      /*   if (1010 < ethercat_loop_counter && ethercat_loop_counter < 2000)
        {
           stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_IDLE;
        }

        if (ethercat_loop_counter > 2000 && !moving_to_position)
        {
           initial_position = stp_in_ptr->stp_actual_position_0;
           target_position = initial_position + 100000;

           stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_POSITION;
           stp_out_ptr->stp_target_position_0 = target_position;

           moving_to_position = TRUE;
        }

        if (moving_to_position)
        {
           if (stp_in_ptr->stp_actual_position_0 >= target_position)
           {
              printf("Target position reached. \n"
                     "Target Position: %d\nActual Position: %d\n",
                     target_position, stp_in_ptr->stp_actual_position_0);

              stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_IDLE;

              printf("Motor is now in idle mode.\n"
                     "opmode: %d\n",
                     stp_out_ptr->stp_requested_opmode_0);

              moving_to_position = FALSE;

              for (int i = 0; i < 1000; i++)
              {
                 if (i % 200 == 0)
                 {
                    printf("opmode: %d\n", stp_out_ptr->stp_requested_opmode_0);
                 }
              }
           }
        } */

      if (ethercat_loop_counter == 10)
      {
         mx2_out_ptr->frequency_reference = 300;
         mx2_out_ptr->command = FORWARD;
      }

      if (10 < ethercat_loop_counter && ethercat_loop_counter == 2000)
      {
         mx2_out_ptr->command = FORWARD;
      }

      if (2000 < ethercat_loop_counter && ethercat_loop_counter < 3000)
      {
         mx2_out_ptr->command = STOP;
      }

      if (3000 < ethercat_loop_counter && ethercat_loop_counter < 5000)
      {
         mx2_out_ptr->command = REVERSE;
      }

      if (ethercat_loop_counter && ethercat_loop_counter > 5000)
      {
         mx2_out_ptr->command = STOP;
      }

      if (ethercat_loop_counter % 200 == 0)
      {
         printf("Motor state is %d\n"
                "Output frequency is: %d\n",
                mx2_in_ptr->status,
                mx2_in_ptr->output_frequency_monitor);
      }

      osal_usleep(5000);
   }

   /* Stop ethercat and close socket */
   ec_close();
}

void simpletest(char *ifname)
{
   int i, oloop, iloop, chk;
   needlf = FALSE;
   inOP = FALSE;
   ec_slave_1_output *out_ptr;
   ec_slave_1_input *in_ptr;
   unsigned int bit_to_set = 0;
   unsigned int loop_count;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */

      if (ec_config_init(FALSE) > 0)
      {
         printf("%d slaves found and configured.\n", ec_slavecount);

         if (forceByteAlignment)
         {
            ec_config_map_aligned(&IOmap);
         }
         else
         {
            ec_config_map(&IOmap);
         }

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0))
            oloop = 1;
         if (oloop > 8)
            oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0))
            iloop = 1;
         if (iloop > 8)
            iloop = 8;

         printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
            out_ptr = (ec_slave_1_output *)ec_slave[1].outputs;
            in_ptr = (ec_slave_1_input *)ec_slave[1].inputs;
            /* cyclic loop */
            while (TRUE)
            {
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);
               loop_count++;

               if (loop_count % 200 == 0 && wkc >= expectedWKC)
               {
                  printf("output command:  %d\n", out_ptr->output_cmd);
                  printf("input command:  %d\n", in_ptr->input_state);
                  printf("loop_count: %d\n", loop_count);

                  bit_to_set = (loop_count / 200) - 1;
                  if (bit_to_set <= 15)
                  {
                     out_ptr->output_cmd = 0;
                     out_ptr->output_cmd |= (1 << bit_to_set);
                     print_binary(out_ptr->output_cmd, 16);
                  }
                  needlf = TRUE;
               }
               osal_usleep(5000);
            }
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (i = 1; i <= ec_slavecount; i++)
            {
               if (ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End simple test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExecute as root\n", ifname);
   }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
         if (needlf)
         {
            needlf = FALSE;
            printf("\n");
         }
         /* one ore more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slave = 1; slave <= ec_slavecount; slave++)
         {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                  ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         if (!ec_group[currentgroup].docheckstate)
            printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
   }
}

int main(int argc, char *argv[])
{
   int retval = 1;
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
      /* start cyclic part */

      initialize_ethercat(argv[1]);

      if (retval < 0)
         return 1;

      ethercat_loop();
      return 1;

      // simpletest(argv[1]);
   }
   else
   {
      ec_adaptert *adapter = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf("\nAvailable adapters:\n");
      adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }

   printf("End program\n");
   return (0);
}
