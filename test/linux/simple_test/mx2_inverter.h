#include <stdint.h>

#define FORWARD 1
#define REVERSE 2
#define STOP 0


typedef struct __attribute__((__packed__))
{
  int16_t command;
  uint16_t frequency_reference;
} mx2_output;

typedef struct __attribute__((__packed__))
{
  int16_t status;
  uint16_t output_frequency_monitor;
} mx2_input;