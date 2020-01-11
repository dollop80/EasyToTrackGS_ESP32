#include "define.h"

uint8_t telem_parse(uint16_t telem_proto, uint8_t b);
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap);