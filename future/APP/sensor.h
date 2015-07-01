/*********************************Copyright (c)*********************************
**
**
**--------------File Info-------------------------------------------------------
** File Name:               sensor.h
** Last modified Date:      
** Last Version:            
** Description:             �����ļ�
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


#ifndef _SENSOR_H_
#define _SENSOR_H_

#ifdef __cplusplus
 extern "C" {
#endif
/******************************************************************************
********************************* �ļ����ò��� ********************************
******************************************************************************/
#include "sim_i2c.h"
//#include "ahrs_dbg.h"
//#include <stdio.h>
#include <stdbool.h>
#include <math.h>    //Keil library 


/******************************************************************************
********************************** �������� ***********************************
******************************************************************************/
#define Kp 2.0f      	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f    	// integral gain governs rate of convergence of gyroscope biases
#define halfT (0.01/2)   // half the sample period

#define M_PI 3.1415926f

//---------------------------------------------------------------------------------------------------
// Variable definitions

//extern float q0,q1,q2,q3;        // quaternion elements representing the estimated orientation
extern float exInt, eyInt, ezInt;        // scaled integral error
extern float fb[3];

/*---------------------*
*    ����I2C��ַ����   * 
*----------------------*/
//#define ADXL345_Addr        0xA6            //���ٶȴ�����������ַ
//#define L3G4200_Addr        0xD2            //�����Ǵ�����������ַ
#define HMC5883L_Addr       0x3C            //�ų�������������ַ
//#define BMP085_Addr         0xee            //��ѹ������������ַ
#define	MPU6050_Addr   		0xD0	  				//����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

/*---------------------*
*    �ж����ȼ�����    * 
*----------------------*/
//#define ADXL345_INT_PRIO    5               //��ֵԽС���ȼ�Խ��
//#define L3G4200D_INT_PRIO   5
//#define HMC5883L_INT_PRIO   5
//#define BMP085_INT_PRIO     5

// ����MPU6050�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

#define Gx_offset -3.06
#define Gy_offset 1.01
#define Gz_offset -0.88

#define Gyro_FS_SEL 0x18

#define Grav_Acc 9800           //�������ٶ�9800ms/s2
//****************************

/******************************************************************************
***************************** HMC5883L �궨�� *********************************
******************************************************************************/
/*---------------------* 
*  HMC5883L�ڲ��Ĵ���  * 
*----------------------*/
#define HMC5883L_REGA   0x00
#define HMC5883L_REGB   0x01
#define HMC5883L_MODE   0x02
#define HMC5883L_HX_H   0x03
#define HMC5883L_HX_L   0x04 
#define HMC5883L_HZ_H   0x05
#define HMC5883L_HZ_L   0x06
#define HMC5883L_HY_H   0x07
#define HMC5883L_HY_L   0x08
#define HMC5883L_STATE  0x09
#define HMC5883L_IRA    0x0a    //�����к�ʹ�õļĴ���
#define HMC5883L_IRB    0x0b
#define HMC5883L_IRC    0x0c 

/*---------------------* 
*   HMC5883 ��������   * 
*----------------------*/
typedef struct
{
    int16_t  m_x;
    int16_t  m_y;
    int16_t  m_z;   
}tg_HMC5883L_TYPE;

typedef struct
{
    int16_t g_x;
    int16_t g_y;
    int16_t g_z;   
}tg_MPU6050_g_TYPE;

typedef struct
{
    int16_t a_x;
    int16_t a_y;
    int16_t a_z;   
}tg_MPU6050_a_TYPE;


typedef struct
{
    float q0;
    float q1;
    float q2;
	float q3; 
	float q0q0;
    float q0q1;
    float q0q2;
    float q0q3;
    float q1q1;
    float q1q2;
    float q1q3;
    float q2q2;   
    float q2q3;
    float q3q3;  	  
}tg_Quaternion_TYPE;//��Ԫ��

/*---------------------* 
*   �ں� ��������      * 
*----------------------*/
typedef struct          //ŷ���Ǳ�ʾ����
{
    float  YAW;       //
    float  PITCH;
    float  ROLL;
	float q0;
    float q1;
    float q2;
	float q3; 
	float Vepx;//��Ե����ƽ̨�ٶ�
	float Vepy;//
	float Vepz;//
	float Repx;//��Ե����ƽ̨λ��
	float Repy;
	float Repz;
	//float q0q0;
    //float q0q1;
    //float q0q2;
    //float q0q3;
    //float q1q1;
    //float q1q2;
    //float q1q3;
    //float q2q2;   
    //float q2q3;
    //float q3q3;  
}tg_EulerAngle_TYPE;
/*---------------------* 
*   HMC5883 У������   * 
*----------------------*/
// Ư��ϵ������λ��1��λ�شų�ǿ��
#define HMC5883L_OFFSET_X   (9)
#define HMC5883L_OFFSET_Y   (149)

//��������
#define HMC5883L_GAIN_X     1f
//#define HMC5883L_GAIN_Y   1.04034582
#define HMC5883L_GAIN_Y     10403     //ʵ��1.04034582,����������� 


/******************************************************************************
****************************** BMP085 �궨�� **********************************
******************************************************************************/
/*---------------------* 
*   BMP085 �ڲ��Ĵ���  * 
*----------------------*/
//none

/*---------------------* 
*   BMP085 ��������    * 
*----------------------*/
typedef struct
{
  int32_t temp;
  int32_t pressure;
  float   altitude;
}tg_BMP085_TYPE;

/*---------------------* 
*   BMP085 ����        * 
*----------------------*/
#define P0_PRESSURE 101325.0f    //��׼����pa


/******************************************************************************
********************************* �ں�����   **********************************
******************************************************************************/



/******************************************************************************
******************************* �Զ���������� ********************************
******************************************************************************/
typedef struct 
{
    float SetSpeed;            //�����趨ֵ
    float ActualSpeed;        //����ʵ��ֵ
    float PID_err;                //����ƫ��ֵ
    float PID_err_last;            //������һ��ƫ��ֵ
    float PID_Kp;
	float PID_Ki;
	float PID_Kd;            //������������֡�΢��ϵ��
    float voltage;            //�����ѹֵ������ִ�����ı�����
    float integral;            //�������ֵ
    float umax;
    float umin;
}tg_PID_TYPE;

/******************************************************************************
******************************** �������Ͷ��� *********************************
******************************************************************************/
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
//��ʼ��������
extern void Sensor_GPIO_Init(void);     //������GPIO�ڳ�ʼ��

extern void HMC5883L_Init(void);

extern void MPU6050_Init(void);

//��ȡ����������


extern void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult);     //��ͨ��������(������20msʱ����ʱ)
extern void HMC5883L_Start(void);                           //����-�ж�-��ȡ (����)��10ms��ʱ
extern void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult); //����-�ж�-��ȡ (��ȡ)

extern void MPU6050_Read_GYRO(tg_MPU6050_g_TYPE *ptResult);
extern void MPU6050_Read_ACCEL(tg_MPU6050_a_TYPE *ptResult);
extern int MPU6050_Read_Temperature(void);

extern void HMC5883L_Calibrate(void);


//��ӡ����������
extern void HMC5883L_Printf(tg_HMC5883L_TYPE * ptResult);
//extern void AHRS_position(tg_Quaternion_TYPE *pt_q,tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros);
extern void AHRS_position(tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros,tg_EulerAngle_TYPE *pt_Euler);
extern void IMU_getYawPitchRoll(tg_EulerAngle_TYPE *pt_Euler);
extern void UART1_ReportMotion(tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros);
extern void UART1_ReportIMU(tg_EulerAngle_TYPE *pt_EulerAngle);
/*************************************************************
************************* �� �� �� �� ************************
*************************************************************/
//none
 
/******************************************************************************
***********************************   END  ************************************
******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif



