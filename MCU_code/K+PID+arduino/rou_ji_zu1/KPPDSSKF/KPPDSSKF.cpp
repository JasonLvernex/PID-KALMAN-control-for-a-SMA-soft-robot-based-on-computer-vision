#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "KPPDSSKF.h"

 
double frand()
{
	return 2 * ((rand() / (double)RAND_MAX) - 0.5);//�������
}


 


/*-------------------------------------------------------------------------------------------------------------*/
/*      
        Q:�������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:��������R���󣬶�̬��Ӧ���������ȶ��Ա��      
*/

double KPM::KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;

        static        double x_last;

        double x_mid = x_last;
        double x_now;

        static        double p_last;

        double p_mid ;
        double p_now;
        double kg;      

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
        kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
        x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
               
        p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance      

        p_last = p_now; //����covarianceֵ
        x_last = x_now; //����ϵͳ״ֵ̬

        return x_now;               
}

void KPM::PIDRegulation(PID *vPID, float processValue)//���PID������ַ�͵�ǰ����ֵ
{
  float thisError;
  thisError=vPID->setpoint-processValue;//��ǰ�������趨ֵ��ȥ��ǰֵ
  vPID->integral+=thisError;//����֣�����������ۼ�����
  vPID->result=vPID->Kp*thisError  +  vPID->Ki*vPID->integral  +  Kd*(vPID->thisError-vPID->lasterror);
  vPID->lasterror=thisError;
}

int KPM::kalmanpid(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,PID *vPID, float demand_gait)
{
	double now_position;
	int width;
	vPID->setpoint=demand_gait;
	now_position=KalmanFilter(ResrcData,ProcessNiose_Q,MeasureNoise_R);//InitialPrediction
	PIDRegulation(vPID, now_position);
	width=vPID->result;
	return width;
}



/*-------------------------------------------------------------------------------------------------------------*/



