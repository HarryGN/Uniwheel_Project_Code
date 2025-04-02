#include "imu.h"
#include "imath.h"
#include "math.h"
#include "mpu6050.h"

_Matrix Mat = {0};
_Attitude att = {0};
#define MahonyPERIOD        5.0f            //��̬�������ڣ�ms��
#define kp 	    0.5f                        //proportional gain governs rate of convergence to accelerometer/magnetometer 
#define ki 	    0.0001f                     //integral gain governs rate of convergenceof gyroscope biases

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      //scaled integral error  


/*
 * ��������mahony_update
 * ����  ����̬����
 * ����  ���������������ݣ���λ������/�룩�����ٶ��������ݣ���λ��g��    
 * ����  ��     
 */
//Gyroscope units are radians/second, accelerometer  units are irrelevant as the vector is normalised.
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) 
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
 
    // If accelerometer readings are zero, return (to avoid division by zero)
    if(ax * ay * az == 0)
        return;
    
    // [ax, ay, az] represents the gravity vector measured by the accelerometer in the body frame (pointing downward)
    norm = invSqrt(ax * ax + ay * ay + az * az); // Normalize accelerometer data
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // VectorA = MatrixC * VectorB
    // VectorA: Reference gravity vector transformed into the body frame
    // MatrixC: The Direction Cosine Matrix (DCM) transforming from the navigation frame to the body frame
    // VectorB: Reference gravity vector in navigation frame (0,0,1)
    // [vx, vy, vz] represents the gravity vector [0,0,1] in the navigation frame, 
    // transformed into the body frame using the DCM.

    vx = Mat.DCM_T[0][2];
    vy = Mat.DCM_T[1][2];
    vz = Mat.DCM_T[2][2];
    
    // Compute error between measured gravity vector and estimated gravity vector
    // The error e is the relative rotation between measured v' and estimated v^.
    // v' represents [ax, ay, az] and v^ represents [vx, vy, vz].
    // This error is used to correct the DCM quaternion.
    // However, this method only corrects the XOY plane misalignment and cannot
    // directly correct yaw drift, which requires gyroscope data fusion.
    // The cross product of two vectors gives the sine of the angle difference,
    // which is approximately linear for small angles.

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
 
    // Integrate error to compensate drift over time
    exInt = exInt + ex * ki;
    eyInt = eyInt + ey * ki;
    ezInt = ezInt + ez * ki;

    // Apply error correction to gyroscope readings
    // The proportional-integral controller (Kp, Ki) adjusts the gyroscope biases
    gx = gx + kp * ex + exInt;
    gy = gy + kp * ey + eyInt;
    gz = gz + kp * ez + ezInt;

    // Update quaternion using first-order Runge-Kutta integration
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * MahonyPERIOD * 0.0005f;
    q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * MahonyPERIOD * 0.0005f;
    q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * MahonyPERIOD * 0.0005f;
    q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * MahonyPERIOD * 0.0005f; 

    // Normalize quaternion to maintain valid rotation representation
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    att.pit = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * rad_to_angle;
    att.rol = asin(2.0f * (q0 * q2 - q1 * q3)) * rad_to_angle;       

    // Integrate gyroscope yaw rate for heading estimation
    att.yaw += Mpu.deg_s.x * MahonyPERIOD * 0.001f;  
}

/*
 * ��������rotation_matrix
 * ����  ����ת���󣺻�������ϵ -> ��������ϵ
 * ����  �� 
 * ����  ��     
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
 * ��������rotation_matrix_T
 * ����  ����ת�����ת�þ��󣺵�������ϵ -> ��������ϵ
 * ����  �� 
 * ����  ��     
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
 * ��������Matrix_ready
 * ����  ���������׼����Ϊ��̬����ʹ��
 * ����  �� 
 * ����  ��     
 */
void Matrix_ready(void)
{
  rotation_matrix();                      //��ת�������
  rotation_matrix_T();                    //��ת�������������
}




