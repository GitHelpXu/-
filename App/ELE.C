#include "ELE.h"
#include "MK60_adc.h"


/////////////��ű�������/////////////
//4�����ֵ
uint16 var[4];
uint16 varE[4];
//��Ⱥ�ȷ��λ��
int8 par,ver;

///////////////���ģ�麯��///////////////
void ELE(){
         //����źŲɼ�
         var[0]=adc_once(AMP1, ADC_12bit);//*0.952294411631;
         var[1]=adc_once(AMP2, ADC_12bit);//*1.5320366132723;
         var[2]=adc_once(AMP3, ADC_12bit);//*0.9692863595302;
         var[3]=adc_once(AMP4, ADC_12bit);//*0.7939171473518;
         //vcan_sendware(var, sizeof(var));
         //��Ź�һ��
         varE[0]=1000.0*(var[0]-adc_min)/ADC;
         varE[1]=1000.0*(var[1]-adc_min)/ADC;
         varE[2]=1000.0*(var[2]-adc_min)/ADC;
         varE[3]=1000.0*(var[3]-adc_min)/ADC;
         if(varE[0]>=100) varE[0]=100;
         if(varE[3]>=100) varE[3]=100;
         //��Ⱥ�
         par=(int)(((varE[3]-varE[0])*1.0/(varE[3]+varE[0])*120)/8);//ˮƽ��У�0��3��
         ver=(int)(((var[2]-var[1])*1.0/(var[2]+var[1])*120)/8);//��ֱ��У�1��2��
         if(par<=-10) par=-10;
         if(par>=10)  par=10;
}