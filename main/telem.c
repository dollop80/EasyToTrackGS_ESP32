#include "define.h"
#include "telem.h"
#include "components/protocols/tiny_ltm.h"
#include "components/protocols/tiny_mavlink.h"
#include "components/protocols/tiny_msp.h"
#include "components/protocols/frskyd.h"
#include "components/protocols/frskyx.h"
#include "components/protocols/pitlab.h"
#include "components/protocols/rvosd.h"

uint8_t telem_parse(uint16_t telem_proto, uint8_t b)
{
	uint8_t res = 0;
	switch(telem_proto)
	{
		case TP_MAVLINK:
			res = mav_parse(b);
		break;
		
	    case TP_LTM:
			res = ltm_parse(b);
		break;
		
		case TP_MSP:
			res = msp_parse(b);
		break;

		case TP_RVOSD:
			res = rvosd_parse(b);
		break;	

		case TP_FRSKY_D:
			res = frskyd_parse(b);
		break;	

		case TP_FRSKY_X:
			res = frskyx_parse(b);
		break;	

		case TP_PITLAB:
			res = pitlab_parse(b);
		break;	

		default:
		break;		
	}
	
	return res;
}

//TODO check if this is the right conversion
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap) {
  // we want convert from frsky to millionth of degrees
  // 0302.7846 -> 03 + (02.7846/60) = 3.04641 -> 3046410
  // first the upper part
  uint32_t first = ((uint32_t)bp / 100) * 100000;
  uint32_t second = ((((uint32_t)bp % 100) * 100000) + ((uint32_t)ap * 10)) / 60;

  // take sign into account
  return ((int32_t)(first + second) * (uint32_t)neg);
}