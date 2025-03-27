#include "imath.h"


// Fast inverse square-root
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/**
  * @brief   ÈýÖáÊý¾Ý½øÐÐ¸³Öµ²Ù×÷
  * @param   *_out_data Êä³öÊý¾ÝÈýÖáÖ¸Õë±äÁ¿
  * @param   *_in_data ÊäÈëÈýÖáÊý¾ÝÖ¸Õë±äÁ¿
  * @retval  x
  */
void _set_val(SI_F_XYZ *_out_data,SI_F_XYZ *_in_data)
{
  _out_data->x = _in_data->x;
  _out_data->y = _in_data->y;
  _out_data->z = _in_data->z;
}
/*
 * º¯ÊýÃû£ºset_value
 * ÃèÊö  £º¸øÊý¾Ý¸³Öµ
 * ÊäÈë  £º_in_datÊäÈëÊý¾ÝÊ×µØÖ·£¬ valueËùÐèÒª¸³µÄÖµ
 * ·µ»Ø  £º     
 */
void  setFloatValue(SI_F_XYZ *_in_data,float value)
{
    _in_data->x = value;
    _in_data->y = value;
    _in_data->z = value;
}
/*
 * º¯ÊýÃû£ºf_abs
 * ÃèÊö  £º¸¡µãÐÍÊý¾Ý¾ø¶ÔÖµ
 * ÊäÈë  £ºf¸¡µãÊý¾Ý 
 * ·µ»Ø  £º¾ø¶ÔÖµ 
 */
float f_abs(float f)
{
	if (f >= 0.0f)
		return f;
	return -f;
}
/**
  * @brief   ¸¡µãÊý¾Ý±È½ÏÅÐ¶Ï½øÐÐ±êÖ¾·µ»Ø
  * @param   *_in_data ÊäÈë±»±È½ÏµÄÈýÖáÊý¾ÝÖ¸Õë±äÁ¿
  * @param   templt Ïà±È½ÏµÄ²Î¿¼Öµ
  * @retval  HAL_OK ÔÚ²Î¿¼Öµ·¶Î§ÄÚ·µ»ØOK£¬HAL_ERROR ²»ÔÚ²Î¿¼·¶Î§ÄÚ·µ»ØERROR
  */
HAL_StatusTypeDef FloatComparison(SI_F_XYZ *_in_data , float templt)
{
  if((f_abs(_in_data->x) > templt) || (f_abs(_in_data->y) > templt) || (f_abs(_in_data->z) > templt))
    return HAL_ERROR;
  return HAL_OK;
}

/**
  * @brief   16Î»ÕûÐÎÊý¾Ý½øÐÐ·¶Î§ÏÞÖÆ
  * @param   thr_in ÐèÒª±»ÏÞÖÆµÄÊäÈëÊý¾Ý
  * @param   thr_min ×îÐ¡Öµ
  * @param   thr_max ×î´óÖµ
  * @retval  ÔÚÏÞÖÆ·¶Î§ÄÚµÄÖµ
  */
uint16_t u16RangeLimit(uint16_t thr_in,uint16_t thr_min,uint16_t thr_max)
{
	if(thr_in < thr_min)	thr_in = thr_min;
	if(thr_in > thr_max)	thr_in = thr_max;
  
	return thr_in;
}
/**
  * @brief   ÕûÐÎÊý¾Ý½øÐÐÈ¡¾ø¶ÔÖµ
  * @param   f ÐèÒª±»È¡¾ø¶ÔÖµµÄÊäÈëÊý¾Ý
  * @retval  ¾ø¶ÔÖµÊý¾Ý
  */
int int_abs(int f)
{
	if (f >= 0)
		return f;
	return -f;
}
