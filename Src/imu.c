#include "imu.h"
#include "imath.h"
#include "math.h"
#include "mpu6050.h"

_Matrix Mat = {0};
_Attitude att = {0};
#define MahonyPERIOD        5.0f            //×ËÌ¬½âËãÖÜÆÚ£¨ms£©
#define kp 	    0.5f                        //proportional gain governs rate of convergence to accelerometer/magnetometer 
#define ki 	    0.0001f                     //integral gain governs rate of convergenceof gyroscope biases

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      //scaled integral error  


/*
 * º¯ÊýÃû£ºmahony_update
 * ÃèÊö  £º×ËÌ¬½âËã
 * ÊäÈë  £ºÍÓÂÝÒÇÈýÖáÊý¾Ý£¨µ¥Î»£º»¡¶È/Ãë£©£¬¼ÓËÙ¶ÈÈýÖáÊý¾Ý£¨µ¥Î»£ºg£©    
 * ·µ»Ø  £º     
 */
//Gyroscope units are radians/second, accelerometer  units are irrelevant as the vector is normalised.
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
 
  if(ax*ay*az==0)
      return;
    
    //[ax,ay,az]ÊÇ»úÌå×ø±êÏµÏÂ¼ÓËÙ¶È¼Æ²âµÃµÄÖØÁ¦ÏòÁ¿(ÊúÖ±ÏòÏÂ)
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//VectorA = MatrixC * VectorB
	//VectorA £º²Î¿¼ÖØÁ¦ÏòÁ¿×ªµ½ÔÚ»úÌåÏÂµÄÖµ
	//MatrixC £ºµØÀí×ø±êÏµ×ª»úÌå×ø±êÏµµÄÐý×ª¾ØÕó  
	//VectorB £º²Î¿¼ÖØÁ¦ÏòÁ¿£¨0,0,1£©      
    //[vx,vy,vz]ÊÇµØÀí×ø±êÏµÖØÁ¦·ÖÏòÁ¿[0,0,1]¾­¹ýDCMÐý×ª¾ØÕó(C(n->b))¼ÆËãµÃµ½µÄ»úÌå×ø±êÏµÖÐµÄÖØÁ¦ÏòÁ¿(ÊúÖ±ÏòÏÂ)    

  vx = Mat.DCM_T[0][2];
  vy = Mat.DCM_T[1][2];
  vz = Mat.DCM_T[2][2];
    
  //»úÌå×ø±êÏµÏÂÏòÁ¿²æ³ËµÃµ½Îó²îÏòÁ¿£¬Îó²îe¾ÍÊÇ²âÁ¿µÃµ½µÄv¡¥ºÍÔ¤²âµÃµ½µÄ v^Ö®¼äµÄÏà¶ÔÐý×ª¡£ÕâÀïµÄv¡¥¾ÍÊÇ[ax,ay,az]¡¯,v^¾ÍÊÇ[vx,vy,vz]¡¯
  //ÀûÓÃÕâ¸öÎó²îÀ´ÐÞÕýDCM·½ÏòÓàÏÒ¾ØÕó(ÐÞÕýDCM¾ØÕóÖÐµÄËÄÔªËØ)£¬Õâ¸ö¾ØÕóµÄ×÷ÓÃ¾ÍÊÇ½«bÏµºÍnÕýÈ·µÄ×ª»¯Ö±µ½ÖØºÏ¡£
  //Êµ¼ÊÉÏÕâÖÖÐÞÕý·½·¨Ö»°ÑbÏµºÍnÏµµÄXOYÆ½ÃæÖØºÏÆðÀ´£¬¶ÔÓÚzÖáÐý×ªµÄÆ«º½£¬¼ÓËÙ¶È¼ÆÎÞ¿ÉÄÎºÎ£¬
  //µ«ÊÇ£¬ÓÉÓÚ¼ÓËÙ¶È¼ÆÎÞ·¨¸ÐÖªzÖáÉÏµÄÐý×ªÔË¶¯£¬ËùÒÔ»¹ÐèÒªÓÃµØ´Å¼ÆÀ´½øÒ»²½²¹³¥¡£
  //Á½¸öÏòÁ¿µÄ²æ»ýµÃµ½µÄ½á¹ûÊÇÁ½¸öÏòÁ¿µÄÄ£ÓëËûÃÇÖ®¼ä¼Ð½ÇÕýÏÒµÄ³Ë»ýa¡Áv=|a||v|sin¦È,
  //¼ÓËÙ¶È¼Æ²âÁ¿µÃµ½µÄÖØÁ¦ÏòÁ¿ºÍÔ¤²âµÃµ½µÄ»úÌåÖØÁ¦ÏòÁ¿ÒÑ¾­¾­¹ýµ¥Î»»¯£¬Òò¶øËûÃÇµÄÄ£ÊÇ1£¬
  //Ò²¾ÍÊÇËµËüÃÇÏòÁ¿µÄ²æ»ý½á¹û½öÓësin¦ÈÓÐ¹Ø£¬µ±½Ç¶ÈºÜÐ¡Ê±£¬²æ»ý½á¹û¿ÉÒÔ½üËÆÓÚ½Ç¶È³ÉÕý±È¡£

  ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;
 
  //¶ÔÎó²îÏòÁ¿½øÐÐ»ý·Ö
	exInt = exInt + ex*ki;
	eyInt = eyInt + ey*ki;
	ezInt = ezInt + ez*ki;

  //×ËÌ¬Îó²î²¹³¥µ½½ÇËÙ¶ÈÉÏ£¬ÐÞÕý½ÇËÙ¶È»ý·ÖÆ¯ÒÆ£¬Í¨¹ýµ÷½ÚKp¡¢KiÁ½¸ö²ÎÊý£¬¿ÉÒÔ¿ØÖÆ¼ÓËÙ¶È¼ÆÐÞÕýÍÓÂÝÒÇ»ý·Ö×ËÌ¬µÄËÙ¶È¡£
	gx = gx + kp*ex + exInt;
	gy = gy + kp*ey + eyInt;
	gz = gz + kp*ez + ezInt;

  //Ò»½×Áú¸ñ¿âËþ·¨¸üÐÂËÄÔªÊý 
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)* MahonyPERIOD * 0.0005f;
	q1 = q1 + ( q0*gx + q2*gz - q3*gy)* MahonyPERIOD * 0.0005f;
	q2 = q2 + ( q0*gy - q1*gz + q3*gx)* MahonyPERIOD * 0.0005f;
	q3 = q3 + ( q0*gz + q1*gy - q2*gx)* MahonyPERIOD * 0.0005f; 

  //°ÑÉÏÊöÔËËãºóµÄËÄÔªÊý½øÐÐ¹éÒ»»¯´¦Àí¡£µÃµ½ÁËÎïÌå¾­¹ýÐý×ªºóµÄÐÂµÄËÄÔªÊý¡£
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
    
  //ËÄÔªËØ×ªÅ·À­½Ç
	att.pit =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * rad_to_angle;
	att.rol =  asin(2.0f*(q0*q2 - q1*q3)) * rad_to_angle;       
 
  //zÖá½ÇËÙ¶È»ý·ÖµÄÆ«º½½Ç
  att.yaw += Mpu.deg_s.z  * MahonyPERIOD * 0.001f;   
}
/*
 * º¯ÊýÃû£ºrotation_matrix
 * ÃèÊö  £ºÐý×ª¾ØÕó£º»úÌå×ø±êÏµ -> µØÀí×ø±êÏµ
 * ÊäÈë  £º 
 * ·µ»Ø  £º     
 */
void rotation_matrix(void)
{
  Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
  Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
  Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);

  Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
  Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
  Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);
   
  Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
  Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
  Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}
/*
 * º¯ÊýÃû£ºrotation_matrix_T
 * ÃèÊö  £ºÐý×ª¾ØÕóµÄ×ªÖÃ¾ØÕó£ºµØÀí×ø±êÏµ -> »úÌå×ø±êÏµ
 * ÊäÈë  £º 
 * ·µ»Ø  £º     
 */
void rotation_matrix_T(void)
{
  Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
  Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);    
  Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2); 
  
  Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
  Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;  
  Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);    
  
  Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
  Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
  Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}
/*
 * º¯ÊýÃû£ºMatrix_ready
 * ÃèÊö  £º¾ØÕó¸üÐÂ×¼±¸£¬Îª×ËÌ¬½âËãÊ¹ÓÃ
 * ÊäÈë  £º 
 * ·µ»Ø  £º     
 */
void Matrix_ready(void)
{
  rotation_matrix();                      //Ðý×ª¾ØÕó¸üÐÂ
  rotation_matrix_T();                    //Ðý×ª¾ØÕóµÄÄæ¾ØÕó¸üÐÂ
}




