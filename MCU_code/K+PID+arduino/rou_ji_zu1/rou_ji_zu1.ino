#include <KPPDSKF.h>
#include <stdlib.h>
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String leg_length;
double leg_length_num;

static float legis_length=0,largest=0;

 union Idata  
{
         float num_d;
//         String num_s;
           char num_s[4];
//         u8        IOArray[512];
};
static Idata mynum,ted,tess;

//使用调度器函数需要用到下面的头文件
//#include <TaskScheduler.h>



//buttonState读取端口6的电平状态，充当开关作用
int buttonState = 1;
int pinstate= 0;
static float width1=100;//=width2=width3=width4

float ResrcData;//computer vision
float ProcessNiose_Q=0.01;  //preset
 float MeasureNoise_R=0.01; //preset  
 float x_now;
 //PID *vPID;
  static PID *VPID; 
 float demand_gait;
 static KPM kppdsskf;
 static int count_loop=0;
 
 static  float thisError;
 static float integral=0;
 static float lasterror=0;
 static float origin=100;
 static float set_point=0;
 //extern KPM kppdssf;
  
//下面的函数只执行一次
void setup() {
    // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  //把四个端口打开
  pinMode(8, OUTPUT);

  //把四个端口设置为低电平
  digitalWrite(8, LOW);

  //设置波特率，其实不用管这部分
  Serial.begin(9600);
 
 //set the PID parameters 


  //PID parameter init
  VPID->setpoint=0;
  VPID->Kp=60;
  VPID->Ki=0.01;
  VPID->Kd=1;
  VPID->integral=0;
  VPID->lasterror=0;
  VPID->result=0;


}

void loop() {
//   if (stringComplete) {
//    //Serial.println(inputString); 
//    //Serial.print(tess.num_d); 
//    // clear the string:
//    inputString = "";
//    stringComplete = false;
//  }
  
  //读取端口6的电平状态
  
//  buttonState = digitalRead(6);
  
  //test of pin 18
//   pinstate = digitalRead(18);
//    Serial.println(pinstate);
    //Serial.println(width1);
    
//legis_length=0;
  //如果开关打开，则端口6为高电平，四个端口按要求输出

    count_loop=count_loop+1;
    //执行被调度的任务，用调度器时放上这一句即可
//    Sch.dispatchTasks();
    demand_gait=calc_demand_gait(count_loop);
//    VPID->setpoint=demand_gait;
set_point=demand_gait;

    ResrcData=largest;//calc_ResrcData(demand_gait,count_loop);
  //  x_now=kppdsskf.KalmanFilter(ResrcData,ProcessNiose_Q,MeasureNoise_R);
        x_now=KalmanFilter(ResrcData,ProcessNiose_Q,MeasureNoise_R);
    //kppdsskf.PIDRegulation(VPID, x_now);

  thisError=set_point-x_now;//锟斤拷前锟斤拷锟斤拷锟斤拷锟借定值锟斤拷去锟斤拷前值
  
  integral+=thisError;//锟斤拷锟斤拷郑锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟桔硷拷锟斤拷锟斤拷
  //limit integral
  if(integral>5)integral=5;
    if(integral<-5)integral=-5;
//  width1=origin+VPID->Kp*thisError  +  VPID->Ki*integral  +  VPID->Kd*(thisError-lasterror);
  width1=origin+10*thisError  +  0.01*integral  +  5*(thisError-lasterror);
  if(thisError<0.5&&thisError>-0.5) width1=origin;
  else origin=width1;
//  Serial.println(thisError);
   //width1=1*thisError  +  0.01*integral  +  0.01*(thisError-lasterror);
  lasterror=thisError;
    
  //  width1=100*kppdsskf.kalmanpid(  ResrcData,//computer vision
  //                   ProcessNiose_Q,  //preset
  //                    MeasureNoise_R,  //preset  
  //                VPID, 
  //               demand_gait) ;//gait
    //limit the amplitude
    
    if(width1<10) width1=0;
    if(width1>1200) width1=1200;
    
    open8();
       pinstate = digitalRead(8);
//        Serial.println(pinstate);
//        Serial.println("duration:");
//        Serial.println(width1);
//        Serial.println("error:");
//        Serial.println(thisError);
//        Serial.println("demanded:");
//        Serial.println(demand_gait);
//          Serial.println("kalman:");
          Serial.println(x_now);
//        Serial.println("cv_read:");
//        Serial.println(ResrcData);
        
    delay(width1);  
//    Serial.print(largest);
    close8();
       pinstate = digitalRead(8);
//////////        Serial.println(width1); Serial.println(x_now);  Serial.println(demand_gait);
        //Serial.print( largest);
   delay(1000);
    largest=legis_length;   //  Serial.print(largest);
    delay(3000);  largest=legis_length;  
    //Serial.print(largest);
    delay(1000); legis_length=0;   delay(5000);   // largest=legis_length;
//    Serial.print(largest);
        legis_length=0;
    
   
}

void open8()
{
  digitalWrite(8, HIGH);
}


void close8()
{
  digitalWrite(8, LOW);
}

double calc_demand_gait(int tim)
{
  double result;
  if(tim<7) {result=27;}
  else result=23;
  return result;
  
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // 下面就是所谓的通讯协议了，如果接收到了换行符号，本例是定义了这次串口通讯数据接收结束//
     // 其他的你还可以加一些规则，比如数据包起始位，规定一个数据包就是以FF为开头的，如果不是就不接收//
     // 也可以加上数据校验位，同样在这里做一个校验判断，如果不符合，则将数据包丢掉，再通过一个标志位写个值，//
     //在主程序里面让发送方重发（主程序里面都写一条IF就行了）//
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    
    if(inputString[0] !='A')
    {
         stringComplete = false;
         inputString="";
    }
//    if(inputString[0] =='A')
//    {
//Serial.print(inputString);
//    }
    
    if(inputString[0] == 'A'&&inputString[1] == 'T'&&inputString[6]=='K'&&inChar == 'L')//&&inputString[6]=='K'&&inChar == 'L'
    {
        char farray[4] = {0};
       stringComplete = true;
       leg_length=inputString.substring(2,6);
//       mynum.num_s=leg_length;
       // leg_length.toCharArray( mynum.num_s,leg_length.length()+1);
       //Serial.println(leg_length);
        leg_length.toCharArray( tess.num_s,leg_length.length()+1);
        //Serial.print(tess.num_d); 
         //Serial.println(tess.num_s);
      //  float *p = (float*)farray;
   // printf("%g\r\n",*p);
        
      // leg_length_num=leg_length.toFloat();
//        Serial.print(inputString);
//        Serial.print(leg_length);
         // Serial.write(farray,4);
         // *(float*)farray = fa;
         // Serial.print(*(float*)farray);
         
             //Serial.write(farray,4);
             
            //strcpy(tess.num_s,farray);
           //  float tee=ByteToFloat(farray);
           if(tess.num_d>legis_length)
           {
           legis_length=tess.num_d;
           }
          // Serial.print( inputString);   
           delay(1000);
               inputString = "";
    stringComplete = false;

       
    }

  }
//     if (stringComplete) {
//    //Serial.println(inputString); 
//    //Serial.print(tess.num_d); 
//    // clear the string:
//    inputString = "";
//    stringComplete = false;
//  }
  
}
float ByteToFloat(unsigned char* byteArry)//使用取地址的方法进行处理
{
return *((float*)byteArry);
}

float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
        float R = MeasureNoise_R;
        float Q = ProcessNiose_Q;

        static        float x_last;

        float x_mid = x_last;
        float x_now;

        static        float p_last;

        float p_mid ;
        float p_now;
        float kg;      

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
        kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
        x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
               
        p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance      

        p_last = p_now; //����covarianceֵ
        x_last = x_now; //����ϵͳ״ֵ̬

        return x_now;               
}
