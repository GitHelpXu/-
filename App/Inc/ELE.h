#ifndef _ELE_H_
#define _ELE_H_


#include "common.h"
#include "MK60_adc.h"



//电磁端口定义
#define AMP1     ADC0_SE17
#define AMP2     ADC0_SE18
#define AMP3     ADC1_SE14
#define AMP4     ADC1_SE15
#define ADC 30000.0
#define adc_min 0.0
#define adc_max 3000.0

//////////////电磁变量定义/////////////
//4个电感值
extern uint16 var[4];
extern uint16 varE[4];
//差比和确定位置
extern int8 par,ver;

extern void ELE();


#endif