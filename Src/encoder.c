#include "encoder.h"
#include "tim.h"


_ENCODER_INFO encoderINFO = { 0 } ;

/**
  * @brief   ±àÂëÖµ´¦Àí£¬½«Ô­±¾·´×ªÓÉ×î´ó¼õÐ¡µÄÕýÊýÊý¾Ý×ª±äÎª´Ó0¿ªÊ¼¼õÐ¡µ½¸ºµÄÊý¾Ý
  * @param   *ecdData ±àÂëÊý¾ÝÖ¸Õë±äÁ¿
  * @retval  x
  */
static void EncoderManage(int *ecdData)
{
  //±àÂëÆ÷·´ÏòÊ±£¬×ªÎªÓëÕýÏòÊ±¶ÔÓ¦µÄ¸ºÊý
  if(*ecdData > FULL_ENCODER * 0.5f)
    *ecdData = *ecdData - FULL_ENCODER ; 
  else 
    *ecdData = *ecdData;  
}

/**
  * @brief   ¶ÁÈ¡±àÂëÆ÷×ªËÙÖµ
  * @param   x
  * @retval  x
  */
uint8_t readEncoderValue(void)
{
  encoderINFO.directionValue = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);      //¶ÁÈ¡±àÂë×ª¶¯µÄ·½Ïò(Õý³£×ª¶¯²ÅÄÜÅÐ¶ÏÕýÈ·£¬´Ë´¦Î´Ê¹ÓÃ£¬µÍËÙ»»Ïò»á¸ÉÈÅÊý¾ÝÅÐ¶Ï)
  encoderINFO.mainNumberValue = __HAL_TIM_GET_COUNTER(&htim2);              //¶ÁÈ¡±àÂëÖµ
  __HAL_TIM_SET_COUNTER(&htim2,0);                                          //Çå³þ±àÂëÖµ
  
  EncoderManage(&encoderINFO.mainNumberValue) ;                             //½«µç»ú·´Ïò×ª¶¯µÄ±àÂëÖµ×ªÎª¸ºµÄÊý¾Ý
  
  return 0;
}





