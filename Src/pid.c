#include "pid.h"



_ALL_PID all;

/**
  * @brief   ´æ´¢pid¿ØÖÆÆ÷²ÎÊý ÈýÐÐÎåÁÐ£¬Ã¿ÐÐ´ú±íÒ»¸öpid¿ØÖÆÆ÷²ÎÊý£¬Ã¿ÁÐ¶ÔÓ¦²ÎÊý 0.kp 1.ki 2.kd 3.»ý·ÖÏÞ·ù  4.pidÊä³öÏÞ·ùÖµ
  * @param   x
  * @retval  x
  */
const float  controller_parameter[3][5] =
{
  /* 0.kp 1.ki 2.kd 3.»ý·ÖÏÞ·ù  4.pidÊä³öÏÞ·ùÖµ */
    {7.5 , 0.0,  0,  550 , 2000 },                           //rol_angle     ÄÚ»·½Ç¶È»·       
    {0.068 , 0.00005,  0.055,  500 , 2000 },                 //vel_encoder   Íâ»·ËÙ¶È»·       
    {26.5 ,  0.0,  0,  500 , 2000 },                         //gyro          ÄÚ»·½ÇËÙ¶È»· 
		
//    {7.5 , 0.0,  0,  550 , 2000 },                           //rol_angle     ÄÚ»·½Ç¶È»·       
//    {0.068 , 0.00005,  0.055,  500 , 2000 },                 //vel_encoder   Íâ»·ËÙ¶È»·       
//    {26.5 ,  0.0,  0,  500 , 2000 },                         //gyro          ÄÚ»·½ÇËÙ¶È»· 
};

/**
  * @brief   PID²ÎÊý³õÊ¼»¯ÅäÖÃ
  * @param   *controller PID¿ØÖÆÆ÷Ö¸Õë£¬Ö¸Ïò²»Í¬µÄ¿ØÖÆÆ÷
  * @param   label PID²ÎÊý±êºÅ£¬Ñ¡Ôñ¶ÔÓ¦¿ØÖÆÆ÷µÄ²ÎÊýÊý×é±êºÅ
  * @retval  x
  */
void pid_init(_PID *controller,uint8_t label)
{
    controller->kp              = controller_parameter[label][0];         
    controller->ki              = controller_parameter[label][1];         
    controller->kd              = controller_parameter[label][2];         
    controller->integral_max    = controller_parameter[label][3];         
    controller->out_max         = controller_parameter[label][4];               
}
//PID²ÎÊý³õÊ¼»¯
void all_pid_init(void)
{
    pid_init(&all.rol_angle,0);
    pid_init(&all.vel_encoder,1);
    pid_init(&all.rol_gyro,2);
} 

/**
  * @brief   PID¿ØÖÆÆ÷
  * @param   *controller PID¿ØÖÆÆ÷Ö¸Õë£¬Ö¸Ïò²»Í¬µÄ¿ØÖÆÆ÷
  * @retval  controller->out ¾­¹ý¿ØÖÆºóµÄÊä³öÖµ
  */
float pid_controller(_PID *controller)
{
    controller->err_last = controller->err;                                                  //±£´æÉÏ´ÎÆ«²î
    controller->err = controller->expect - controller->feedback;                            //Æ«²î¼ÆËã
    controller->integral += controller->ki * controller->err;                               //»ý·Ö  
    //»ý·ÖÏÞ·ù
    if(controller->integral >  controller->integral_max)     controller->integral =  controller->integral_max;
    if(controller->integral < -controller->integral_max)     controller->integral = -controller->integral_max;
    //pidÔËËã
    controller->out =  controller->kp*controller->err + controller->integral + controller->kd*(controller->err-controller->err_last);
   
    //Êä³öÏÞ·ù
    if(controller->out >  controller->out_max)   controller->out =  controller->out_max;
    if(controller->out < -controller->out_max)   controller->out = -controller->out_max;
    return controller->out;
}
/**
  * @brief   PID¿ØÖÆÆ÷»ý·ÖÏîÇå³ý
  * @param   *controller PID¿ØÖÆÆ÷Ö¸Õë£¬Ö¸Ïò²»Í¬µÄ¿ØÖÆÆ÷
  * @retval  x
  */
void clear_integral(_PID *controller)
{
    controller->integral = 0.0f;
}











