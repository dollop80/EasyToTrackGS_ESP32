#include <stdint.h>

#define SYN_UART 0x5a
typedef struct
{
    uint8_t len; 
	uint8_t cnt;
	uint8_t f_syn;
	uint8_t crc;
	uint8_t msg[60];
} _sb_proto;

uint8_t packPacket(uint8_t type, uint8_t *out_buff, uint8_t *in_buff, uint8_t cnt);
uint8_t parseChar(_sb_proto * inst, uint8_t c);