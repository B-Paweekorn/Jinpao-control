
#ifndef INC_MCU_MSGS_H_
#define INC_MCU_MSGS_H_

#include <stdlib.h>

typedef union {
  double f64;
  uint8_t u8[8];
} msgs_convert;

typedef struct {
  msgs_convert *buff;
  size_t buff_size;
  uint8_t header;
  uint8_t carry_bit;
  char state;
  uint8_t counter;
  uint8_t checksum;
  bool status;
} MCU_msgs;

void MCU_msgs_Init(MCU_msgs *msgs, size_t buff_size, uint8_t header, uint8_t carry_bit);

void encode_msg2buffer(MCU_msgs *msgs, uint8_t* txbuff, size_t txbuff_size);

void decode_buffer2msg(MCU_msgs *msgs, uint8_t val);

#endif