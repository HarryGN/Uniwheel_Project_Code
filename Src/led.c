
#include "led.h"
#include "string.h"

led_set_str run_led; //ÔËÐÐÖ¸Ê¾µÆ


/*****************************************************************************
** º¯ÊýÃû³Æ: led_ctl 
** ¹¦ÄÜÃèÊö: led ÁÁÃð¿ØÖÆ
** ÊäÈë²ÎÊý: opt ledÑ¡Ôñ
** ÊäÈë²ÎÊý: cmd ledÁÁÃðÃüÁî
** Êä³ö²ÎÊý£ºÎÞ
** ·µ »Ø Öµ£ºÎÞ
*****************************************************************************/
static void led_ctl(uint8_t opt,uint8_t cmd)
{
	switch(opt)
	{
		case RUN_LED:
			 RUN_LED_RUN(cmd);
		break;
//		case CAN_LED:
//			 CAN_LED_RUN(cmd);
//		break;
		default:break;
	}
}

/*****************************************************************************
** º¯ÊýÃû³Æ: led_operation(led_set_str * led_data)
** ¹¦ÄÜÃèÊö: led ÔËÐÐº¯Êý£¬¼ÆÊýÆ÷ÔÚÒ»¸öÁÁÃðÖÜÆÚÄÚÀÛ¼Ó£¬¸ù¾ÝÕ¼¿Õ±È¾ö¶¨ÁÁÃðÊ±¼ä
** ÊäÈë²ÎÊý: led ÉèÖÃ½á¹¹Ìå±äÁ¿
** Êä³ö²ÎÊý£ºÎÞ
** ·µ »Ø Öµ£ºÎÞ
*****************************************************************************/
void led_operation(led_set_str * led_data)
{
	led_data->heart++;
	if(led_data->heart> 0 && led_data->heart < led_data->pulse)
	{
		led_ctl(led_data->led_opt,ON);
	}
	else if(led_data->heart >= led_data->pulse && led_data->heart < led_data->period)
	{
		led_ctl(led_data->led_opt,OFF);
	}
	else if(led_data->heart >= led_data->period)
	{
		led_data->heart = 0;
		
		if(led_data->num != 0)
		{
			if(++led_data->count >= led_data->num)
			{
				memset(led_data , 0 , sizeof(led_set_str));
			}
		}
	}
}

/*****************************************************************************
** º¯ÊýÃû³Æ: led_set
** ¹¦ÄÜÃèÊö: ledµÆÉÁË¸ÉèÖÃ
** ÊäÈë²ÎÊý: uint8_t opt led Ñ¡Ôñ
** ÊäÈë²ÎÊý: uint16_t period ÉÁË¸ÖÜÆÚ
** ÊäÈë²ÎÊý: uint16_t duty_cycle ÁÁ¶ÈÕ¼¿Õ±È
** ÊäÈë²ÎÊý: uint16_t num ÉÁË¸´ÎÊý,Èç¹ûÊÇ0ÔòÎÞÏÞÉÁË¸
** Êä³ö²ÎÊý£ºÎÞ
** ·µ »Ø Öµ£ºÎÞ
*****************************************************************************/
void led_set(uint8_t opt,uint16_t period, uint16_t pulse,uint16_t num)
{
	led_set_str led_data;
	led_data.led_opt = opt;
	led_data.period = period;
	led_data.pulse = pulse;
	led_data.num = num;
	led_data.heart = 0;
	led_data.count = 0;
	switch(opt)
	{
		case RUN_LED:
			memcpy(&run_led,&led_data,sizeof(led_set_str));
		break;
//		case CAN_LED:
//			memcpy(&can_led,&led_data,sizeof(led_set_str));
//		break;
		
		default:break;
	}
}

