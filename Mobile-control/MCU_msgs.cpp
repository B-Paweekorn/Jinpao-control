
#include "MCU_msgs.h"

void create_msgs_buffer(msgs_convert **buffer, size_t size);

void MCU_msgs_Init(MCU_msgs *msgs, size_t buff_size, uint8_t header, uint8_t carry_bit) {
  msgs->buff_size = buff_size;
  msgs->header = header;
  msgs->carry_bit = carry_bit;
  msgs->state = 'H';
  msgs->counter = 0;
  msgs->checksum = 0;
  msgs->status = 0;
  create_msgs_buffer(&(msgs->buff), buff_size);
}

void encode_msg2buffer(MCU_msgs *msgs, uint8_t *txbuff, size_t txbuff_size) {
  if (txbuff_size < (msgs->buff_size * 8) + 4) {
    msgs->status = 0;
  } else {
    txbuff[0] = msgs->header;
    txbuff[1] = ~msgs->header;
    txbuff[2] = msgs->header;
    for (int i = 0; i < msgs->buff_size; i++) {
      for (int j = 0; j < 8; j++) {
        txbuff[3 + (8 * i) + j] = msgs->buff[i].u8[j];
        txbuff[(msgs->buff_size * 8) + 3] += msgs->buff[i].u8[j];
      }
    }
    txbuff[(msgs->buff_size * 8) + 3] /= 256;
    txbuff[(msgs->buff_size * 8) + 3] += msgs->carry_bit;
    txbuff[(msgs->buff_size * 8) + 3] = ~txbuff[(msgs->buff_size * 8) + 3];
    msgs->status = 1;
  }
}

void decode_buffer2msg(MCU_msgs *msgs, uint8_t val) {
  switch (msgs->state) {
    case 'H':
      msgs->status = 0;
      if (val == msgs->header) {
        msgs->state = '1';
      }
      break;
    case '1':
      // msgs->checksum = 2;
      if (val == 0xff - msgs->header) {
        msgs->state = '2';
      } else msgs->state = 'H';
      break;
    case '2':
      // msgs->checksum = 1;
      if (val == msgs->header) {
        msgs->state = 'R';
        msgs->counter = 0;
        msgs->checksum = 0;
      } else msgs->state = 'H';
      break;
    case 'R':
      msgs->buff[msgs->counter / 8].u8[msgs->counter % 8] = val;
      msgs->checksum += val;
      msgs->counter++;
      if (msgs->counter == msgs->buff_size * 8) msgs->state = 'E';
      break;
    case 'E':
      msgs->checksum = msgs->checksum / 256;
      msgs->checksum += msgs->carry_bit;
      msgs->checksum = ~msgs->checksum;
      msgs->status = msgs->checksum == val;
      msgs->state = 'H';
      break;
  }
}

void create_msgs_buffer(msgs_convert **buffer, size_t buff_size) {
  *buffer = (msgs_convert *)malloc(buff_size * 8);
}