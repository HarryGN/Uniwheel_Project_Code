#include "filter.h"


/*
 * º¯ÊýÃû£ºbutterworth_lpf
 * ÃèÊö  £º¶þ½×°ÍÌØÎÖË¹ÂË²¨Æ÷Ô­ÐÍ
 * ÊäÈë  £ºnow_inputÊäÈëÊý¾Ý£¬ bufferÖÐ¼äÊý¾Ý»º´æ£¬parameterÂË²¨²ÎÊý
 * ·µ»Ø  £ºÂË²¨Ö®ºóµÄÊý¾Ý     
 */
float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter)
{
    buffer->input_data[2] = now_input;

    /* Butterworth LPF */
    buffer->output_data[2] =   parameter->b[0] * buffer->input_data[2]
                             + parameter->b[1] * buffer->input_data[1]
                             + parameter->b[2] * buffer->input_data[0]
                             - parameter->a[1] * buffer->output_data[1]
                             - parameter->a[2] * buffer->output_data[0];
    /* x(n) ±£´æ */
    buffer->input_data[0] = buffer->input_data[1];
    buffer->input_data[1] = buffer->input_data[2];
    /* y(n) ±£´æ */
    buffer->output_data[0] = buffer->output_data[1];
    buffer->output_data[1] = buffer->output_data[2];
  
    return buffer->output_data[2];
}








