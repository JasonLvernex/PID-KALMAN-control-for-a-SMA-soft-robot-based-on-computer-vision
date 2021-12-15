#ifndef _KPPDSSKF_H
#define _KPPDSSKF_H

#if ARDUINO >= 100
#include "Arduino.h"   
#else
#include "WProgram.h"  
#endif




#define RETURN_ERROR 0
#define RETURN_NORMAL 1



struct _1_ekf_filter
{
	float LastP;
	float	Now_P;
	float out;
	float Kg;
	float Q;
	float R;	
};
    typedef struct
    {
      float  setpoint;       //�趨ֵ
      float Kp;     //����ϵ��
      float Kd;      //���ϵ��
      float Ki;    //΢��ϵ��
      float integral;//���ֵ
      float lasterror;     //ǰһ��ƫ��
      float result; //���ֵ
    }PID;
    
class KPM
{
//private:
public:
double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
void PIDRegulation(PID *vPID, float processValue);
int kalmanpid(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,PID *vPID, float demand_gait);
} ;
//static KPM kppdssf;

#endif
