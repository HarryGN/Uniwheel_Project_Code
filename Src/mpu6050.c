#include "mpu6050.h"
#include "cmsis_os.h"
#include "filter.h"
#include "soft_iic.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "stdio.h"
#include "flash.h"
#include "imath.h"

S16_XYZ  acc_raw = {0};                  //���ٶȼ�ԭʼ���ݴ洢
S16_XYZ  gyro_raw = {0};                 //������ԭʼ���ݴ洢
SI_F_XYZ  gyro_raw_cal = {0};                 //����������У׼��ԭʼ���ݴ洢
SI_F_XYZ acc_raw_f = {0};
SI_F_XYZ gyro_raw_f = {0};
SI_F_XYZ acc_att_lpf = {0};
SI_F_XYZ gyro_lpf = {0};
SI_F_XYZ gyro_offset = {0,0,0} ;         //��������ƫ���ݴ洢
_Mpu6050_data Mpu = {0};
_GYRO_CAL CalGyro = {0};                 //������У׼��ر����洢

/* Ӳ��iic���֣�ʵ��ʹ������ģ��iic���ɲ����в鿴�� BEGIN */
static void I2Cx_Error(uint8_t Addr)
{
	HAL_I2C_DeInit(&hi2c1);
	MX_I2C1_Init();
}

int Sensors_I2C_WriteRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len, unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,data_ptr, len,I2Cx_FLAG_TIMEOUT); 
	/* ���ͨѶ״̬ */
	if(status != HAL_OK)
	{
		/* ���߳������� */
		I2Cx_Error(slave_addr);
	}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(&hi2c1, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* �ȴ�������� */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len, unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status =HAL_I2C_Mem_Read(&hi2c1,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,data_ptr,len,I2Cx_FLAG_TIMEOUT);    
	/* ���ͨѶ״̬ */
	if(status != HAL_OK)
	{
		/* ���߳������� */
		I2Cx_Error(slave_addr);
	}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(&hi2c1, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* �ȴ�������� */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}
/* Ӳ��iic���� END */

/**
  * @brief   д���ݵ�MPU6050�Ĵ���
  * @param   reg_add:�Ĵ�����ַ
	* @param	 reg_data:Ҫд�������
  * @retval  
  */
void MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
  IIC_Write_One_Byte(MPU6050_ADDRESS ,reg_add ,reg_dat);
}

/**
  * @brief   ��MPU6050�Ĵ�����ȡ����
  * @param   reg_add:�Ĵ�����ַ
	* @param	 Read���洢���ݵĻ�����
	* @param	 num��Ҫ��ȡ��������
  * @retval  x
  */
void MPU6050_ReadData(uint8_t reg_add,unsigned char* Read,uint8_t num)
{
  for(uint8_t i = 0; i < num ; i++ )
  {
    * ( Read + i ) = IIC_Read_One_Byte(MPU6050_ADDRESS,reg_add);   
    reg_add ++;  
  }
}

/*
 * ��������mpu6050_init
 * ����  ����ʼ��MOU6050����
 * ����  ��x    
 * ����  ��x 
 */
void mpu6050_init(void)
{  
  HAL_Delay(100);  
	MPU6050_WriteReg(PWR_MGMT_1, 0x80);	          	
  HAL_Delay(100);  
	MPU6050_WriteReg(PWR_MGMT_1, 0x00);           //����mpu		
  /* when DLPF is disabled( DLPF_CFG=0 or 7),���������Ƶ�� = 8kHz; 
     when DLPFis enabled,���������Ƶ�� = 1KHz 
     fs(����Ƶ��) = ���������Ƶ�� / (1 + SMPLRT_DIV)*/	
	MPU6050_WriteReg(SMPLRT_DIV, 0x00);		        //sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz	
	MPU6050_WriteReg(MPU_CONFIG, 0x03);           //�ڲ���ͨ  acc:44hz	gyro:42hz
	MPU6050_WriteReg(GYRO_CONFIG, 0x18);			    // gyro scale  ��+-2000��/s
	MPU6050_WriteReg(ACCEL_CONFIG, 0x10);			    // Accel scale ��+-8g (65536/16=4096 LSB/g)    
  HAL_Delay(100);  	
}
/**
  * @brief   ��ȡMPU6050��ID
  * @param   x
  * @retval  ��������1���쳣����0
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
  MPU6050_ReadData(WHO_AM_I,&Re,1);    //��������ַ
	if(Re != 0x68 && Re != 0x98)
	{
		printf("MPU6050 dectected error!\r\n��ⲻ��MPU6050ģ�飬����ģ���뿪����Ľ���");
		return 0;
	}
	else
	{
		printf("MPU6050 ID = %d\r\n",Re);
		return 1;
	}
}

/**
  * @brief   ��ȡMPU6050�ļ��ٶ�����
  * @param   *accData ������ٶ�����ָ�����
  * @retval  x
  */
void MPU6050ReadAcc(S16_XYZ *accData)
{
  uint8_t buf[6];
  MPU6050_ReadData(ACCEL_XOUT_H, buf, 6);
  accData->x = (buf[0] << 8) | buf[1];
  accData->y = (buf[2] << 8) | buf[3];
  accData->z = (buf[4] << 8) | buf[5];
}

/**
  * @brief   ��ȡMPU6050�ĽǼ��ٶ�����
  * @param   *gyroData ����������ָ�����
  * @retval  x
  */
void MPU6050ReadGyro(S16_XYZ *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(GYRO_XOUT_H,buf,6);
    gyroData->x = (buf[0] << 8) | buf[1];
    gyroData->y = (buf[2] << 8) | buf[3];
    gyroData->z = (buf[4] << 8) | buf[5];
}

/*
 * ��������get_acc_raw
 * ����  ����ȡ���ٶȼ�����ԭʼ����
 * ����  ��     
 * ����  ��     
 */
void get_acc_raw(void)
{
  MPU6050ReadAcc(&acc_raw);
  acc_raw_f.x = (float)acc_raw.x;
  acc_raw_f.y = (float)acc_raw.y;
  acc_raw_f.z = (float)acc_raw.z;
}
/*
 * ��������gyro_30hz_parameter
 * ����  ��������˹��ͨ�˲�������
 * ����  ��     
 * ����  ��     
 */
_Butterworth_parameter gyro_30hz_parameter =
{
  //200hz---30hz
  1,  -0.7477891782585,    0.272214937925,
  0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   gyro_butter_data[3];

/*
 * ��������get_gyro_raw
 * ����  ����ȡ����������ԭʼ���� & ��ƫУ׼ȥ�� & ��ͨ�˲�
 * ����  ��     
 * ����  ��     
 */
void get_gyro_raw(void)
{
  MPU6050ReadGyro(&gyro_raw);
  /* ��ԭʼ��������תΪ���������� */
  gyro_raw_cal.x = (float) gyro_raw.x;
  gyro_raw_cal.y = (float) gyro_raw.y;
  gyro_raw_cal.z = (float) gyro_raw.z;
  /* ԭʼ���ݼ�ȥУ׼��ƫ���� */
  gyro_raw.x = gyro_raw.x - gyro_offset.x;
  gyro_raw.y = gyro_raw.y - gyro_offset.y;
  gyro_raw.z = gyro_raw.z - gyro_offset.z;        
  /* ������ٶ�ԭʼ���ݽ����˲�������תΪ�������� */  
  gyro_raw_f.x = (float)butterworth_lpf( ( (float) gyro_raw.x) , &gyro_butter_data[0] , &gyro_30hz_parameter );
  gyro_raw_f.y = (float)butterworth_lpf( ( (float) gyro_raw.y) , &gyro_butter_data[1] , &gyro_30hz_parameter );
  gyro_raw_f.z = (float)butterworth_lpf( ( (float) gyro_raw.z) , &gyro_butter_data[2] , &gyro_30hz_parameter );
}
/*
 * ��������get_iir_factor
 * ����  ����ȡIIR�˲������˲�����
 * ����  ��out_factor�˲������׵�ַ��Time����ִ�����ڣ�Cut_Off�˲���ֹƵ��     
 * ����  ��     
 */
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
	*out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}
/**
  * @brief   IIR��ͨ�˲���
  * @param   *acc_in ������������ָ�����
  * @param   *acc_out �����������ָ�����
  * @param   lpf_factor �˲�����
  * @retval  x
  */
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor)
{
	acc_out->x = acc_out->x + lpf_factor * (acc_in->x - acc_out->x); 
	acc_out->y = acc_out->y + lpf_factor * (acc_in->y - acc_out->y); 
	acc_out->z = acc_out->z + lpf_factor * (acc_in->z - acc_out->z); 
}
/**
  * @brief   ���ٶȼ��˲�����
  */
_Butterworth_parameter acc_5hz_parameter =
{
  //200hz---1hz
//  1,   -1.955578240315,   0.9565436765112,
//  0.000241359049042, 0.000482718098084, 0.000241359049042
  //200hz---2hz
//  1,   -1.911197067426,   0.9149758348014,
//  0.0009446918438402,  0.00188938368768,0.0009446918438402
    //200hz---5hz
    1,                  -1.778631777825,    0.8008026466657,
    0.005542717210281,   0.01108543442056,  0.005542717210281
    //200hz---10hz
//    1,   -1.561018075801,   0.6413515380576,
//    0.02008336556421,  0.04016673112842,  0.02008336556421
    //200hz---15hz
//    1,   -1.348967745253,   0.5139818942197,
//    0.04125353724172,  0.08250707448344,  0.04125353724172
    //200hz---20hz
//    1,    -1.14298050254,   0.4128015980962,
//    0.06745527388907,   0.1349105477781,  0.06745527388907
    //200hz---30hz
//    1,  -0.7477891782585,    0.272214937925,
//    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   acc_butter_data[3];
/*
 * ��������acc_butterworth_lpf
 * ����  ��������˹���ٶȵ�ͨ�˲�
 * ����  ��acc_in�˲�ǰ�ļ��ٶ��׵�ַ��acc_out�˲����������ٶ������׵�ַ     
 * ����  ��     
 */
void acc_butterworth_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
    acc_out->x = butterworth_lpf(acc_in->x,&acc_butter_data[0],&acc_5hz_parameter);
    acc_out->y = butterworth_lpf(acc_in->y,&acc_butter_data[1],&acc_5hz_parameter);
    acc_out->z = butterworth_lpf(acc_in->z,&acc_butter_data[2],&acc_5hz_parameter);    
}
/*
 * ��������get_acc_g
 * ����  ��ԭʼ���ٶ�תΪ�������ٶ�gΪ��λ����
 * ����  ��acc_inԭʼ�ļ��ٶ��׵�ַ��acc_out��gΪ��λ�ļ��ٶ������׵�ַ     
 * ����  ��     
 */
void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
	acc_out->x = (float)(acc_in->x * acc_raw_to_g);
	acc_out->y = (float)(acc_in->y * acc_raw_to_g);
	acc_out->z = (float)(acc_in->z * acc_raw_to_g);
}
/*
 * ��������get_rad_s
 * ����  ��ԭʼ����������תΪ����/��Ϊ��λ������
 * ����  ��gyro_inԭʼ�������������׵�ַ��gyro_out��rad/sΪ��λ�������������׵�ַ     
 * ����  ��     
 */
void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out)
{
	gyro_out->x = (float)(gyro_in->x * gyro_raw_to_radian_s);
	gyro_out->y = (float)(gyro_in->y * gyro_raw_to_radian_s);
	gyro_out->z = (float)(gyro_in->z * gyro_raw_to_radian_s);
}
/*
 * ��������get_deg_s
 * ����  ��ԭʼ����������תΪ��/��Ϊ��λ������
 * ����  ��gyro_inԭʼ�������������׵�ַ��gyro_deg_out��deg/sΪ��λ�������������׵�ַ     
 * ����  ��     
 */
void get_deg_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_deg_out)
{
	gyro_deg_out->x = (float)(gyro_in->x * gyro_raw_to_deg_s);
	gyro_deg_out->y = (float)(gyro_in->y * gyro_raw_to_deg_s);
	gyro_deg_out->z = (float)(gyro_in->z * gyro_raw_to_deg_s);    
}
/*
 * ��������gyro_cal
 * ����  ����������ƫ����У׼
 * ����  ��gyro_inԭʼ�ľ�ֹʱ���������������׵�ַ    
 * ����  ��     
 */
void gyro_cal(SI_F_XYZ *gyro_in)
{
  float pFlashRead[3];
  if(CalGyro.flag==1 && 1)                                    
  {
    if(CalGyro.i < GyroCalSumValue)		                                        //ԭʼ��ֹ���ݶ���ۼ���ȡƽ����GyroCalSumValueΪ�ۼӴ���
    {                       
      CalGyro.offset.x += gyro_in->x; 
      CalGyro.offset.y += gyro_in->y;
      CalGyro.offset.z += gyro_in->z;
      CalGyro.i++;
    }
    else
    {
      CalGyro.i = 0;
      CalGyro.OffsetFlashWrite.x = CalGyro.offset.x / GyroCalSumValue;        //�õ�����ľ�ֹ��ƫ���� 
      CalGyro.OffsetFlashWrite.y = CalGyro.offset.y / GyroCalSumValue;        //�õ�����ľ�ֹ��ƫ���� 
      CalGyro.OffsetFlashWrite.z = CalGyro.offset.z / GyroCalSumValue;        //�õ�����ľ�ֹ��ƫ���� 

      /* ����������ƫд��flash */
      FLASH_WriteThreeFloatData(SENSOR_CAL_ADDRESS, CalGyro.OffsetFlashWrite.x,
                                                    CalGyro.OffsetFlashWrite.y,
                                                    CalGyro.OffsetFlashWrite.z);
      /* У׼������֮��������ȡ��������ʹ�� */
      FLASH_ReadFloatData(SENSOR_CAL_ADDRESS,&pFlashRead[0],3);
      printf("pCal��%f  %f  %f \r\n",pFlashRead[0],pFlashRead[1],pFlashRead[2]);
      CalGyro.OffsetFlashRead.x = pFlashRead[0] ; 
      CalGyro.OffsetFlashRead.y = pFlashRead[1] ; 
      CalGyro.OffsetFlashRead.z = pFlashRead[2] ; 
      
      /* �ж��Ƿ���ȷ���� */
      if(FloatComparison( &CalGyro.OffsetFlashRead, 300.0f) == HAL_OK)
      {
        _set_val(&gyro_offset , &CalGyro.OffsetFlashRead);                  //��������ƫʹ������
        CalGyro.success = 1;                                                //У׼�ɹ�
      }
      else 
      {
        setFloatValue(&gyro_offset , 0.0f); 
        CalGyro.success = 2;                                                //У׼ʧ���򲻽�����ƫ��ֵ���� 
      }
      CalGyro.offset.x = 0;                                                 //���У׼��ƫ����
      CalGyro.offset.y = 0;                                                 //���У׼��ƫ����
      CalGyro.offset.z = 0;                                                 //���У׼��ƫ����  
      CalGyro.flag = 0;                                                     //���У׼��׼λ
    }
  }
}

/*
 * ��������ReadCalData
 * ����  ����������ƫУ׼���ݽ��ж�ȡ
 * ����  ��      
 * ����  ��err     
 */
int readCalData(void)
{
  float pFlashRead[3];
  ErrorStatus err;
  
  /* ÿ���ϵ��ȡ����У׼���ݽ���ʹ�� */
  FLASH_ReadFloatData(SENSOR_CAL_ADDRESS,&pFlashRead[0],3);
  printf("calFLASH reading \r\n");
  printf("calFLASH reading \r\n");
  printf("calFLASH reading \r\n");
  printf("pCal��%f  %f  %f \r\n",pFlashRead[0],pFlashRead[1],pFlashRead[2]);
  
  CalGyro.OffsetFlashRead.x = pFlashRead[0] ; 
  CalGyro.OffsetFlashRead.y = pFlashRead[1] ; 
  CalGyro.OffsetFlashRead.z = pFlashRead[2] ; 
  
  /* �ж��Ƿ���ȷ���� */
  if(FloatComparison( &CalGyro.OffsetFlashRead, 300.0f) == HAL_OK)              //У׼��ȷ����д����ƫ����
  {
    _set_val(&gyro_offset, &CalGyro.OffsetFlashRead);                           //��������ƫ����
    err = SUCCESS ;
    printf("calFLASH READ SUCCEED��\r\n");
  }
  else                    
  {
    setFloatValue(&gyro_offset, 0.0f);                                          //У׼����д��0
    err = ERROR;
    printf("calFLASH READ ERR\r\n");
    printf("calFLASH READ ERR\r\n");
    printf("calFLASH READ ERR\r\n");
  }
        
  return err;
}
