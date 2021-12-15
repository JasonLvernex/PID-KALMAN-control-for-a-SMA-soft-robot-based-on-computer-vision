function [o]=tryPID(yd,ye,k,error_P,error_D,error_I)
kP=5;kI=0.01;kD=1;
global error_P

global error_D

global error_I


if k<2
    k=2;
end
error_P(k)=yd-ye(k-1);
error_D(k)=error_P(k)-error_P(k-1);
error_I(k)=error_I(k)+error_I(k-1);

o=kP*error_P(k)+kI*error_I(k)+kD*error_D(k);

end