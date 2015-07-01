/*********************************Copyright (c)*********************************
**
**
**--------------File Info-------------------------------------------------------
** File Name:               sensor.h
** Last modified Date:      
** Last Version:            
** Description:             配置文件
**
**------------------------------------------------------------------------------
** Created By:              wanxuncpx
** Created date:            
** Version:                 
** Descriptions:            
**
*******************************************************************************/

/******************************************************************************
更新说明:
    
******************************************************************************/

/******************************************************************************
*********************************  应 用 资 料 ********************************
******************************************************************************/


#ifndef _SENSOR_H_
#define _SENSOR_H_

#ifdef __cplusplus
 extern "C" {
#endif
/******************************************************************************
********************************* 文件引用部分 ********************************
******************************************************************************/
#include "sim_i2c.h"
//#include "ahrs_dbg.h"
//#include <stdio.h>
#include <stdbool.h>
#include <math.h>    //Keil library 


/******************************************************************************
********************************** 参数配置 ***********************************
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
*    器件I2C地址定义   * 
*----------------------*/
//#define ADXL345_Addr        0xA6            //加速度传感器器件地址
//#define L3G4200_Addr        0xD2            //陀螺仪传感器器件地址
#define HMC5883L_Addr       0x3C            //磁场传感器器件地址
//#define BMP085_Addr         0xee            //气压传感器器件地址
#define	MPU6050_Addr   		0xD0	  				//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

/*---------------------*
*    中断优先级分配    * 
*----------------------*/
//#define ADXL345_INT_PRIO    5               //数值越小优先级越高
//#define L3G4200D_INT_PRIO   5
//#define HMC5883L_INT_PRIO   5
//#define BMP085_INT_PRIO     5

// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
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

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)

#define Gx_offset -3.06
#define Gy_offset 1.01
#define Gz_offset -0.88

#define Gyro_FS_SEL 0x18

#define Grav_Acc 9800           //重力加速度9800ms/s2
//****************************

/******************************************************************************
***************************** HMC5883L 宏定义 *********************************
******************************************************************************/
/*---------------------* 
*  HMC5883L内部寄存器  * 
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
#define HMC5883L_IRA    0x0a    //读序列号使用的寄存器
#define HMC5883L_IRB    0x0b
#define HMC5883L_IRC    0x0c 

/*---------------------* 
*   HMC5883 数据类型   * 
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
}tg_Quaternion_TYPE;//四元数

/*---------------------* 
*   融合 数据类型      * 
*----------------------*/
typedef struct          //欧拉角表示方法
{
    float  YAW;       //
    float  PITCH;
    float  ROLL;
	float q0;
    float q1;
    float q2;
	float q3; 
	float Vepx;//相对地球的平台速度
	float Vepy;//
	float Vepz;//
	float Repx;//相对地球的平台位移
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
*   HMC5883 校正参数   * 
*----------------------*/
// 漂移系数。单位：1单位地磁场强度
#define HMC5883L_OFFSET_X   (9)
#define HMC5883L_OFFSET_Y   (149)

//比例因子
#define HMC5883L_GAIN_X     1f
//#define HMC5883L_GAIN_Y   1.04034582
#define HMC5883L_GAIN_Y     10403     //实际1.04034582,这里便于整除 


/******************************************************************************
****************************** BMP085 宏定义 **********************************
******************************************************************************/
/*---------------------* 
*   BMP085 内部寄存器  * 
*----------------------*/
//none

/*---------------------* 
*   BMP085 数据类型    * 
*----------------------*/
typedef struct
{
  int32_t temp;
  int32_t pressure;
  float   altitude;
}tg_BMP085_TYPE;

/*---------------------* 
*   BMP085 参数        * 
*----------------------*/
#define P0_PRESSURE 101325.0f    //标准海拔pa


/******************************************************************************
********************************* 融合数据   **********************************
******************************************************************************/



/******************************************************************************
******************************* 自定义参数配置 ********************************
******************************************************************************/
typedef struct 
{
    float SetSpeed;            //定义设定值
    float ActualSpeed;        //定义实际值
    float PID_err;                //定义偏差值
    float PID_err_last;            //定义上一个偏差值
    float PID_Kp;
	float PID_Ki;
	float PID_Kd;            //定义比例、积分、微分系数
    float voltage;            //定义电压值（控制执行器的变量）
    float integral;            //定义积分值
    float umax;
    float umin;
}tg_PID_TYPE;

/******************************************************************************
******************************** 数据类型定义 *********************************
******************************************************************************/
//none

/******************************************************************************
********************************* 数 据 声 明 *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:由外提供   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:向外提供   * 
*----------------------*/
//none

/******************************************************************************
********************************* 函 数 声 明 *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:由外提供   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:向外提供   * 
*----------------------*/
//初始化传感器
extern void Sensor_GPIO_Init(void);     //传感器GPIO口初始化

extern void HMC5883L_Init(void);

extern void MPU6050_Init(void);

//读取传感器数据


extern void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult);     //普通完整测量(有至少20ms时序延时)
extern void HMC5883L_Start(void);                           //启动-中断-读取 (启动)有10ms延时
extern void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult); //启动-中断-读取 (读取)

extern void MPU6050_Read_GYRO(tg_MPU6050_g_TYPE *ptResult);
extern void MPU6050_Read_ACCEL(tg_MPU6050_a_TYPE *ptResult);
extern int MPU6050_Read_Temperature(void);

extern void HMC5883L_Calibrate(void);


//打印传感器数据
extern void HMC5883L_Printf(tg_HMC5883L_TYPE * ptResult);
//extern void AHRS_position(tg_Quaternion_TYPE *pt_q,tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros);
extern void AHRS_position(tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros,tg_EulerAngle_TYPE *pt_Euler);
extern void IMU_getYawPitchRoll(tg_EulerAngle_TYPE *pt_Euler);
extern void UART1_ReportMotion(tg_HMC5883L_TYPE *pt_compass,tg_MPU6050_a_TYPE *pt_accel,tg_MPU6050_g_TYPE *pt_gyros);
extern void UART1_ReportIMU(tg_EulerAngle_TYPE *pt_EulerAngle);
/*************************************************************
************************* 其 它 杂 项 ************************
*************************************************************/
//none
 
/******************************************************************************
***********************************   END  ************************************
******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif



