#include "simple_bin.h"

uint8_t packPacket(uint8_t type, uint8_t *out_buff, uint8_t *in_buff, uint8_t cnt)
{
	uint8_t crc, i;
	uint8_t *p;
	
	p=out_buff;
	*p++=SYN_UART;
	*p++=cnt;
	crc=cnt;
	*p++=type;
	crc^=type;
	for(i=0; i<cnt; i++)
	  {
		  crc^=in_buff[i];
		  *p++=in_buff[i];
	  }
	*p++=crc;
	*p++='\r';
	*p='\n';  
	return 1+1+1+cnt+1+2;
}

uint8_t parseChar(_sb_proto * inst, uint8_t c) 
{
	uint8_t result = 0;

	if(inst->f_syn==0) 
	{
	if(c==SYN_UART) 
		inst->f_syn=1; 
	}
	else if(inst->f_syn==1)
	{
	if(c<2 || c>60) inst->f_syn=0;
	else { inst->len=c; inst->crc=c; inst->cnt=0; inst->f_syn=2; }
	}
	else
	{
	if(inst->cnt > inst->len)
	  {
	  inst->f_syn=0;  
	  if(inst->crc==c)
		{
		   result = inst->len+1;
		}
	  }
	else  {inst->msg[inst->cnt++]=c; inst->crc^=c; }
	}   
	return result;  
}