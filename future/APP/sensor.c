/*********************************Copyright (c)*********************************
**                               
**
**--------------File Info-------------------------------------------------------
** File Name:               hmc5883.c
** Last modified Date:      
** Last Version:            
** Description:             ��ά�����Ǵ�����
**
**------------------------------------------------------------------------------
** Created By:              wanxuncpx
** Created date:            
** Version:                 
** Descriptions:            
**
*******************************************************************************/

/******************************************************************************
����˵��:
    
******************************************************************************/

/******************************************************************************
*********************************  Ӧ �� �� �� ********************************
******************************************************************************/

/******************************************************************************
********************************* �ļ����ò��� ********************************
******************************************************************************/
#include "sensor.h"


/******************************************************************************
******************************* �Զ���������� ********************************
******************************************************************************/

#include <math.h>

//----------------------------------------------------------------------------------------------------
// Definitions

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

float T[3][3];
float fb[3];

/******************************************************************************
********************************* �� �� �� �� *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:�����ṩ   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:�����ṩ   * 
*----------------------*/
//none

/******************************************************************************
********************************* �� �� �� �� *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:�����ṩ   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:�����ṩ   * 
*----------------------*/
//none

extern void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);

/******************************************************************************
*********************************  ����ʼ  **********************************
******************************************************************************/


/******************************************************************************
/ ��������:��ʼ��HMC5883
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void HMC5883L_Init(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //��������ģʽ
}



/******************************************************************************
/ ��������:��ȡHMC5883������
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult)
{
    uint8_t tmp[6];
    int32_t s32Val;
    
    Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
    Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //��������ģʽ
    Delayms(10);
    
    tmp[0]=Single_Read(HMC5883L_Addr,HMC5883L_HX_H);//OUT_X_L_A
    tmp[1]=Single_Read(HMC5883L_Addr,HMC5883L_HX_L);//OUT_X_H_A
    
    tmp[2]=Single_Read(HMC5883L_Addr,HMC5883L_HZ_H);//OUT_Z_L_A
    tmp[3]=Single_Read(HMC5883L_Addr,HMC5883L_HZ_L);//OUT_Z_H_A
    
    tmp[4]=Single_Read(HMC5883L_Addr,HMC5883L_HY_H);//OUT_Y_L_A
    tmp[5]=Single_Read(HMC5883L_Addr,HMC5883L_HY_L);//OUT_Y_H_A

    ptResult->m_x  = (int16_t)((tmp[0] << 8) | tmp[1])+HMC5883L_OFFSET_X;
    s32Val = (int16_t)((tmp[4] << 8) | tmp[5])+HMC5883L_OFFSET_Y;    
    s32Val = (s32Val*HMC5883L_GAIN_Y)/10000;
    ptResult->m_y    = (int16_t)s32Val;
    ptResult->m_z    = (int16_t)((tmp[2] << 8) | tmp[3]);
}
void MPU6050_Init(void)
{
   	Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	//��Դ��������ֵ��0x00(��������),�������״̬
	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x01);    //�����ǲ����ʣ�01   500Hz
	Single_Write(MPU6050_Addr,CONFIG, 0x01); 		//��ͨ�˲�Ƶ�ʣ�0x01 186Hz
	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01);	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
}
/******************************************************************************
/ ��������:��ȡMPU6050��GYRO����
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void MPU6050_Read_GYRO(tg_MPU6050_g_TYPE *ptResult)
{
    uint8_t tmp[6];

	tmp[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L); 
	tmp[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);
    
    tmp[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);
    tmp[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
    
    tmp[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);
    tmp[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);

    ptResult->g_x  = ((int16_t)(((tmp[1] << 8) | tmp[0])));///16.4);//16.4
	ptResult->g_y  = ((int16_t)(((tmp[3] << 8) | tmp[2])));///16.4);
	ptResult->g_z  = ((int16_t)(((tmp[5] << 8) | tmp[4])));///16.4);
}
/******************************************************************************
/ ��������:��ȡMPU6050��ACCEL����
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void MPU6050_Read_ACCEL(tg_MPU6050_a_TYPE *ptResult)
{
    uint8_t tmp[6];
	
	tmp[0]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 
	tmp[1]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
    
    tmp[2]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);
    tmp[3]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
    
    tmp[4]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_L);
    tmp[5]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_H);

    ptResult->a_x  = ((int16_t)(((tmp[1] << 8) | tmp[0])));///1.67);//1.0
	ptResult->a_y  = ((int16_t)(((tmp[3] << 8) | tmp[2])));///1.67);
	ptResult->a_z  = ((int16_t)(((tmp[5] << 8) | tmp[4])));///1.67);
}

int MPU6050_Read_Temperature(void)
{
	int temperature = 0;
	char temperatureH = 0 , temperatureL = 0;
	temperatureH = Single_Read(MPU6050_Addr,TEMP_OUT_H);
	temperatureL = Single_Read(MPU6050_Addr,TEMP_OUT_L);
	temperature = ((int16_t)(((temperatureH<<8)|temperatureL))/34.00)+365.3; //  temp/340+365.3
	return temperature;
}


/******************************************************************************
/ ��������:����HMC5883��ʼת��(����������-�ж�-��ȡ���ݵĳ���)
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (����)
******************************************************************************/
void HMC5883L_Start(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //��������ģʽ
}

/******************************************************************************
/ ��������:��ȡHMC5883������(����������-�ж�-��ȡ���ݵĳ���)
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (��ѯ)
******************************************************************************/
void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult)
{
    uint8_t tmp[6];
    int32_t s32Val;

    if(true == Mult_Read(HMC5883L_Addr,HMC5883L_HX_H,tmp,6))   //�����������������
    {
        //��������(����x������y�����)
        ptResult->m_x  = (int16_t)((tmp[0] << 8) | tmp[1])+HMC5883L_OFFSET_X;
        s32Val = (int16_t)((tmp[4] << 8) | tmp[5])+HMC5883L_OFFSET_Y;    
        s32Val = (s32Val*HMC5883L_GAIN_Y)/10000;
        ptResult->m_y    = (int16_t) s32Val;//(int16_t)((tmp[4] << 8) | tmp[5])+HMC5883L_OFFSET_Y; //(int16_t)s32Val;
        ptResult->m_z    = (int16_t)((tmp[2] << 8) | tmp[3]);
    }
} 

/******************************************************************************
/ ��������:HMC5883У׼
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (��ѯ)
******************************************************************************/
void HMC5883L_Calibrate(void)
{
	Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x15);   //30Hz,�����Լ�ģʽ
	Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x01);   //��һ����ģʽ
	Delayms(10);
	Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);
	Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //�ص�����ģʽ
}


float invSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;          // get bits for floating value
    i =  0x5f375a86 - (i>>1);    // gives initial guess
    x = *(float*)&i;            // convert bits back to float
    x = x * (1.5f - xhalf*x*x); // Newton step
    return x;
}

void AHRS_position(tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros,tg_EulerAngle_TYPE *pt_Euler)
{
	float normalise;
 	float ax, ay, az;
    float mx, my, mz;
	float gx, gy, gz;
    float hx,hy,hz,bx,bz;
    float ex, ey, ez;
	float vx, vy, vz, wx, wy, wz;

    // auxiliary variables to reduce number of repeated operations
    float q0q0 = q0*q0;	
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

	ax=(float)pt_accel->a_x;
	ay=(float)pt_accel->a_y;
	az=(float)pt_accel->a_z;

	mx=(float)pt_compass->m_x;
	my=(float)pt_compass->m_y;
	mz=(float)pt_compass->m_z;

	gx=(float)((pt_gyros->g_x * 4000/65536)* M_PI/180);
	gy=(float)((pt_gyros->g_y * 4000/65536)* M_PI/180);
	gz=(float)((pt_gyros->g_z * 4000/65536)* M_PI/180);

    normalise = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * normalise;
    ay = ay * normalise;
    az = az * normalise;
    normalise = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * normalise;
    my = my * normalise;
    mz = mz * normalise; 

	//compute reference direction of flux
    hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
    hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
    hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;      

	//estimated direction of gravity and flux (v and w)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
    wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
    wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

	//error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	if((ex != 0.0f) && (ey != 0.0f) && (ez != 0.0f))
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;        
		ezInt = ezInt + ez * Ki * halfT;
		
		// adjusted gyroscope measurements
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;	
	}
/************
	//integral error scaled integral gain
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
	
	//adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
**************/
	pt_Euler->YAW= -atan2(2 * q1q2 + 2 * q0q3, -2 * q1q1 - 2 * q3q3 + 1)* (180 / 3.14159265);//180/M_PI; // yaw
	if(pt_Euler->YAW <   0) pt_Euler->YAW += 360.0f;  //�� -+180��  ת��0-360��
	if(pt_Euler->YAW > 360) pt_Euler->YAW -= 360.0f;  //�� -+180��  ת��0-360��
	pt_Euler->PITCH= -asin(-2 * q1q3 + 2 * q0q2)* (180 / 3.14159265); // pitch
	pt_Euler->ROLL= atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1)* (180 / 3.14159265); // roll
		
	T[0][0]= q0q0 + q1q1 + q2q2 + q3q3;   
	T[0][1]= 2 * (q1q2 - q0q3);   
	T[0][2]= 2 * (q1q3 + q0q1);   
	T[1][0]= 2 * (q1q2 + q0q3);   
	T[1][1]= q0q0 - q1q1 + q2q2 - q3q3;   
	T[1][2]= 2 * (q2q3 - q0q1);   
	T[2][0]= 2 * (q1q3 - q0q2);   
	T[2][1]= 2 * (q2q3 + q0q1);   
	T[2][2]= q0q0 - q1q1 - q2q2 + q3q3;
	//����������T�����˵õ����ٶ�
	fb[0] =  T[0][0] * gx + T[0][1] * gy + T[0][2] * gz;
	fb[1] =  T[1][0] * gx + T[1][1] * gy + T[1][2] * gz;
	fb[2] =  T[2][0] * gx + T[2][1] * gy + T[2][2] * gz; 

	pt_Euler->Repx += fb[0] * halfT * halfT * 4;
	pt_Euler->Repy += fb[1] * halfT * halfT * 4;
	pt_Euler->Repz += fb[2] * halfT * halfT * 4;
	
	//integrate quaternion rate and normalise
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	//normalise quaternion
    normalise = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * normalise;
    q1 = q1 * normalise;
    q2 = q2 * normalise;
    q3 = q3 * normalise;
	
						 	   
}
#if 0
void AHRS_position(tg_Quaternion_TYPE *pt_q,tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros)
{
	double normalise;
 	double ax, ay, az;
    double mx, my, mz;
	double gx, gy, gz;
    double hx,hy,hz,bx,bz;
    double ex, ey, ez;
	double vx, vy, vz, wx, wy, wz;

    // auxiliary variables to reduce number of repeated operations
    pt_q->q0q0 = pt_q->q0 * pt_q->q0;
    pt_q->q0q1 = pt_q->q0 * pt_q->q1;
    pt_q->q0q2 = pt_q->q0 * pt_q->q2;
    pt_q->q0q3 = pt_q->q0 * pt_q->q3;
    pt_q->q1q1 = pt_q->q1 * pt_q->q1;
    pt_q->q1q2 = pt_q->q1 * pt_q->q2;
    pt_q->q1q3 = pt_q->q1 * pt_q->q3;
    pt_q->q2q2 = pt_q->q2 * pt_q->q2;   
    pt_q->q2q3 = pt_q->q2 * pt_q->q3;
    pt_q->q3q3 = pt_q->q3 * pt_q->q3;   

	ax=(double)pt_accel->a_x;
	ay=(double)pt_accel->a_y;
	az=(double)pt_accel->a_z;

	mx=(double)pt_compass->m_x;
	my=(double)pt_compass->m_y;
	mz=(double)pt_compass->m_z;

	gx=(double)((pt_gyros->g_x * 4000/65535) * (M_PI/180));
	gy=(double)((pt_gyros->g_y * 4000/65535) * (M_PI/180));
	gz=(double)((pt_gyros->g_z * 4000/65535) * (M_PI/180));

    //normalise = invSqrt(ax*ax + ay*ay + az*az);    
	normalise = 1.0 / sqrt(ax * ax + ay * ay + az * az);   
    ax = ax * normalise;
    ay = ay * normalise;
    az = az * normalise;
    //normalise = invSqrt(mx*mx + my*my + mz*mz);   
	normalise = 1.0 / sqrt(mx * mx + my * my + mz * mz);       
    mx = mx * normalise;
    my = my * normalise;
    mz = mz * normalise; 

	//compute reference direction of flux
    hx = 2 * mx * (0.5 - pt_q->q2q2 - pt_q->q3q3) + 2 * my * (pt_q->q1q2 - pt_q->q0q3) + 2 * mz * (pt_q->q1q3 + pt_q->q0q2);
    hy = 2 * mx * (pt_q->q1q2 + pt_q->q0q3) + 2 * my * (0.5 - pt_q->q1q1 - pt_q->q3q3) + 2 * mz * (pt_q->q2q3 - pt_q->q0q1);
    hz = 2 * mx * (pt_q->q1q3 - pt_q->q0q2) + 2 * my * (pt_q->q2q3 + pt_q->q0q1) + 2 * mz * (0.5 - pt_q->q1q1 - pt_q->q2q2);         
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;      

	//estimated direction of gravity and flux (v and w)
    vx = 2 * (pt_q->q1q3 - pt_q->q0q2);
    vy = 2 * (pt_q->q0q1 + pt_q->q2q3);
    vz = pt_q->q0q0 - pt_q->q1q1 - pt_q->q2q2 + pt_q->q3q3;
    wx = 2 * bx * (0.5 - pt_q->q2q2 - pt_q->q3q3) + 2 * bz * (pt_q->q1q3 - pt_q->q0q2);
    wy = 2 * bx * (pt_q->q1q2 - pt_q->q0q3) + 2 * bz * (pt_q->q0q1 + pt_q->q2q3);
    wz = 2 * bx * (pt_q->q0q2 + pt_q->q1q3) + 2 * bz * (0.5 - pt_q->q1q1 - pt_q->q2q2);  

	//error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	if((ex != 0.0f) && (ey != 0.0f) && (ez != 0.0f))
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;        
		ezInt = ezInt + ez * Ki * halfT;
		
		// adjusted gyroscope measurements
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;	
	}
/************
	//integral error scaled integral gain
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;


	
	//adjusted gyroscope measurements
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
**************/
	
	//integrate quaternion rate and normalise
    q0 = q0 + (- pt_q->q1 * gx - pt_q->q2 * gy - pt_q->q3 * gz) * halfT;
    q1 = q1 + (pt_q->q0 * gx + pt_q->q2 * gz - pt_q->q3 * gy) * halfT;
    q2 = q2 + (pt_q->q0 * gy - pt_q->q1 * gz + pt_q->q3 * gx) * halfT;
    q3 = q3 + (pt_q->q0 * gz + pt_q->q1 * gy - pt_q->q2 * gx) * halfT;  
	
	//normalise quaternion
    //normalise = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	normalise = 1.0 / sqrt(pt_q->q0 * pt_q->q0 + pt_q->q1 * pt_q->q1 + pt_q->q2 * pt_q->q2 + pt_q->q3 * pt_q->q3);
    pt_q->q0 = pt_q->q0 * normalise;
    pt_q->q1 = pt_q->q1 * normalise;
    pt_q->q2 = pt_q->q2 * normalise;
    pt_q->q3 = pt_q->q3 * normalise;									 	   
}
#endif
  
/**************************ʵ�ֺ���********************************************
*����ԭ��:           void IMU_getYawPitchRoll(float * angles)
*��������:         ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/
void IMU_getYawPitchRoll(tg_EulerAngle_TYPE * pt_EulerAngle) 
{
	float angles[3];
	//volatile float gx=0.0, gy=0.0, gz=0.0; //������������
	//IMU_getQ(q); //����ȫ����Ԫ��
	 
	angles[0]= -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q1*q1 - 2 * q3 * q3 + 1)* (1800 / 3.14159265);//180/M_PI; // yaw
	if(angles[0] <    0)angles[0] += 3600.0f;  //�� -+180��  ת��0-360��
	if(angles[0] > 3600)angles[0] -= 3600.0f;  //�� -+180��  ת��0-360��
	pt_EulerAngle->YAW  = 	(int16_t)angles[0];
	
	//angles[1]= asin(2 * q1 * q3 + 2 * q0 * q1)* (1800 / 3.14159265); // pitch
	//pt_EulerAngle->PITCH =	(int16_t)angles[1];
 
	//angles[2]= -atan2(-2 * q0 * q2 + 2 * q1 * q3, -2 * q1 * q1 - 2 * q2 * q2 + 1)* (1800 / 3.14159265); // roll
	//pt_EulerAngle->ROLL =	(int16_t)angles[2];

	angles[1]= -asin(-2 * q1 * q3 + 2 * q0 * q2)* (1800 / 3.14159265); // pitch
	pt_EulerAngle->PITCH =	(int16_t)angles[1];
 
	angles[2]= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* (1800 / 3.14159265); // roll
	pt_EulerAngle->ROLL =	(int16_t)angles[2];
}


void PID_init(tg_PID_TYPE * pid)
{
    pid->SetSpeed=0.0;
    pid->ActualSpeed=0.0;
    pid->PID_err=0.0;
    pid->PID_err_last=0.0;
    pid->voltage=0.0;
    pid->integral=0.0;
    pid->PID_Kp=0.2;
	pid->PID_Ki=0.1;       //ע�⣬���ϼ�����ȣ�����Ӵ��˻��ֻ��ڵ�ֵ
    pid->PID_Kd=0.2;
    pid->umax=400;
    pid->umin=-200;
}
float PID_realize(tg_PID_TYPE * pid, float speed)
{
    int index;
	float PID_err_abs;
    pid->SetSpeed=speed;
    pid->PID_err=pid->SetSpeed-pid->ActualSpeed;
	if(pid->PID_err<0)
	{
		PID_err_abs=0-pid->PID_err;
	}
	else PID_err_abs=pid->PID_err;

	if(pid->ActualSpeed>pid->umax)         //��ɫ��ɫ��ʾ�����ֱ��͵�ʵ��
    {
		if(PID_err_abs>200)                    //��ɫ��עΪ���ַ������
        {
            index=0;
        }
		else
		{
            index=1;
            if(pid->PID_err<0)
            {
            	pid->integral+=pid->PID_err;
            }
        }
    }
	else if(pid->ActualSpeed<pid->umin)
	{
        if(PID_err_abs>200)                    //���ַ������
        {
            index=0;
        }
		else
		{
            index=1;
            if(pid->PID_err>0)
            {
				pid->integral+=pid->PID_err;
            }
        }
    }
	else
	{
        if(PID_err_abs>200)                    //���ַ������
        {
            index=0;
        }
		else
		{
            index=1;
            pid->integral+=pid->PID_err;
        }
    }
    pid->voltage=pid->PID_Kp * pid->PID_err+index * pid->PID_Ki * pid->integral+pid->PID_Kd * (pid->PID_err - pid->PID_err_last);
    pid->PID_err_last=pid->PID_err;
    //pid->ActualSpeed=pid->voltage*1.0;
    return pid->voltage;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART1->SR & USART_FLAG_TXE));
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
				,int16_t alt,int16_t tempr,int16_t press)
*��������:		����λ�����;�����������̬����
���������
		int16_t yaw ���������ĺ���Ƕȡ���λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
		int16_t pitch ����õ��ĸ����Ƕȣ���λ 0.1�ȡ�-900 - 900 ��Ӧ -90.0 -> 90.0 ��
		int16_t roll  �����õ��ĺ���Ƕȣ���λ0.1�ȡ� -1800 -> 1800 ��Ӧ -180.0  ->  180.0��
		int16_t alt   ��ѹ�߶ȡ� ��λ0.1�ס�  ��Χһ�����ͱ���
		int16_t tempr �¶� �� ��λ0.1���϶�   ��Χ��ֱ����ĵ�·�岻����������
		int16_t press ��ѹѹ������λ10Pa  һ������ѹǿ��101300pa ����Ѿ�����һ�����͵ķ�Χ����Ҫ����10�ٷ�����λ��
		int16_t IMUpersec  ��̬�������ʡ�����IMUpersecÿ�롣
���������û��	
*******************************************************************************/
void UART1_ReportIMU(tg_EulerAngle_TYPE *pt_EulerAngle)
{
 	unsigned int temp=0xaF+2;
	char ctemp;
	int16_t yaw		= (int)(10*pt_EulerAngle->YAW);
	int16_t pitch	= (int)(10*pt_EulerAngle->PITCH);
	int16_t roll	= (int)(10*pt_EulerAngle->ROLL);
	int16_t alt=1;
	int16_t tempr=1;
	int16_t press=1;

	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+2);
	UART1_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=alt;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
*��������:		����λ�����͵�ǰ�����������ֵ
���������
	int16_t ax  ���ٶ� X��ADC��� ��Χ ��һ���з�������
	int16_t ay  ���ٶ� Y��ADC��� ��Χ ��һ���з�������
	int16_t az  ���ٶ� Z��ADC��� ��Χ ��һ���з�������
	int16_t gx  ������ X��ADC��� ��Χ ��һ���з�������
	int16_t gy  ������ Y��ADC��� ��Χ ��һ���з�������
	int16_t gz  ������ Z��ADC��� ��Χ ��һ���з�������
	int16_t hx  ������ X��ADC��� ��Χ ��һ���з�������
	int16_t hy  ������ Y��ADC��� ��Χ ��һ���з�������
	int16_t hz  ������ Z��ADC��� ��Χ ��һ���з�������
	
���������û��	
*******************************************************************************/
void UART1_ReportMotion(tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros)
{	
	unsigned int temp=0xaF+9;
	char ctemp;
	int16_t ax, ay, az;
    int16_t mx, my, mz;
	int16_t gx, gy, gz;

	ax=(float)pt_accel->a_x;
	ay=(float)pt_accel->a_y;
	az=(float)pt_accel->a_z;

	mx=(float)pt_compass->m_x;
	my=(float)pt_compass->m_y;
	mz=(float)pt_compass->m_z;

	gx=(float)pt_gyros->g_x;
	gy=(float)pt_gyros->g_y;
	gz=(float)pt_gyros->g_z;

	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+8);
	UART1_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(mx<0)mx=32768-mx;
	ctemp=mx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=mx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(my<0)my=32768-my;
	ctemp=my>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=my;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(mz<0)mz=32768-mz;
	ctemp=mz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=mz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}








