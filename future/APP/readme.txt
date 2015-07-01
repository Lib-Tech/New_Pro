/****************************************************************************
* Copyright (C), 2011 奋斗嵌入式工作室 ourstm.5d6d.com
*
* 本例程在 奋斗版STM32开发板Tiny上调试通过           
* QQ: 9191274, 旺旺：sun68, Email: sun68@163.com 
* 淘宝店铺：ourstm.taobao.com  
*
* 文件名: app.c
* 内容简述:
*       本例程操作系统采用ucos2.86a版本， 建立了5个任务
			任务名											 优先级
			APP_TASK_START_PRIO                               2	        主任务	  		
            Task_Com1_PRIO                                    4			COM1通信任务
            Task_Led1_PRIO                                    7			LED1 闪烁任务
            Task_Led2_PRIO                                    8			LED2 闪烁任务
            Task_Led3_PRIO                                    9			LED3 闪烁任务
		 当然还包含了系统任务：
		    OS_TaskIdle                  空闲任务-----------------优先级最低
			OS_TaskStat                  统计运行时间的任务-------优先级次低
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2011-05-26 sun68  创建该文件
*


基于奋斗版STM32 Tiny （NRF24L01转USB串口接口板）
奋斗STM32开发板论坛：http://ourstm.5d6d.com	
                     http://www.ourstm.net
奋斗STM32 QQ讨论4群: 133971340（仅限奋斗板用户）
奋斗STM32 QQ讨论3群（高级群）: 115132365（满）
奋斗STM32 QQ讨论1群（高级群）: 42465044（满）
奋斗STM32 QQ讨论2群（高级群）: 105680620（满）
*/