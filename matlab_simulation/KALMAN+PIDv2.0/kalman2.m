%Kalman filter+PID
%x=Ax+B(u+w(k));
%y=Cx+D+v(k)
clear all;
close all;

ts=0.001;
M=3000;

%Continuous Plant  ????
a=25;b=133;
sys=tf(b,[1,a,0]);        %?????????
dsys=c2d(sys,ts,'z');     %????????
[num,den]=tfdata(dsys,'v');%??????????????

A1=[0 1;0 -a];
B1=[0;b];
C1=[1 0];
D1=[0];
[A,B,C,D]=c2dm(A1,B1,C1,D1,ts,'z');%%????LTI?????????????

Q=1;           %Covariances of w     ????Q
R=1;           %Covariances of v     ????R

P=B*Q*B';          %Initial error covariance  ???????
x=zeros(2,1);      %Initial condition on the state  ???????

ye=zeros(M,1); %%???y?
global error_P
 error_P=zeros(M,1);
global error_D
error_D=zeros(M,1);
global error_I
error_I=zeros(M,1);


ycov=zeros(M,1);

u_1=0;u_2=0;
y_1=0;y_2=0;
% for tim=1:1:100
    %%yd(tim)=1.0*sin(2*pi*1.5*tim*ts)
%     yd(tim)=100;
%     width=tryPID(yd,ye,tim,error_P,error_D,error_I)
for k=1:1:M
time(k)=k;%*ts

w(k)=1*rands(1);   %Process noise on u      ??
v(k)=1*rands(1);   %Measurement noise on y   ??

%frequency domain: X(s)=Vpp/s*(1-e^(-Ks))
%transfer to time domain: rectpuls(t,w)
%%u(k)=1.0*sin(2*pi*1.5*k*ts);
    yd(k)=200;
    width=tryPID(yd(k),ye,k,error_P,error_D,error_I)

u(k)=width();
u(k)=u(k)+w(k);                % ??????
  
y(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;
yv(k)=y(k)+v(k);% ??????

%Measurement update
   
   %Time update
    x=A*x+B*u(k);               %??????
    P=A*P*A'+B*Q*B';            %???????
    
    %Measurement update
    Mn=P*C'/(C*P*C'+R);         %????????
    x=A*x+Mn*(yv(k)-C*A*x);     %??????
    P=(eye(2)-Mn*C)*P;          %???????
    
    errcov(k)=C*P*C';        %  Covariance of estimation error  ????????
      ye(k)=C*x+D;           %Filtered value   ??????
      
	 u_2=u_1;u_1=u(k);
  	 y_2=y_1;y_1=ye(k);
end
figure(1);
plot(time,y,'r',time,yv,'k:',time,yd,'b-','linewidth',1);
xlabel('time(s)');ylabel('y,yv')
legend('ideal signal','signal with noise','demanded gait');
% figure(1);
% plot(time,y,'r','linewidth',2);
% xlabel('time(s)');ylabel('y')
% legend('ideal signal');


figure(2);
plot(time,y,'r',time,ye,'k:','linewidth',2);
xlabel('time(s)');ylabel('y,ye')
legend('ideal signal','filtered signal');

figure(3);
plot(time,errcov,'k','linewidth',2);
xlabel('time(s)');ylabel('Covariance of estimation error');

figure(4);
plot(time,u,'k','linewidth',1);
xlabel('time(s)');ylabel('control value');

% waitforbuttonpress;
% end


