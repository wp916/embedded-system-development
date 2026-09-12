#ifndef __REAL_MAIN_H__
#define __REAL_MAIN_H__

#include "cmsis_os.h"		//定时器相关句柄定义
#include "freertos.h"
#include "main.h"

#include "YS_moto.h"
#include "cather.h"

#if M2006_DEBUG
void M2006_SET_PID(uint16_t kp,uint16_t ki,uint16_t kd,int16_t IS);
#endif
#if YS_DEBUG
void YS_Moto_SET_PID(uint16_t kp,uint16_t ki,uint16_t kd,uint16_t IS);
#endif

#endif
