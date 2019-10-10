function [q] = ik(p)
%FK Summary of this function goes here
%   Detailed explanation goes here

[robot, serial]=initializer("ax18");

% gettign position component
x=p(1);
y=p(2);
z=p(3);

% using ls for formula transcribing
l1=17;
l2=17;
l3=7;
l4=4;
l5=4;
l6=9;

% getting theta 1
t1=atan2(y,x);

   
a=atan2(l4,l3+l5+l6);
ct3a=(x^2+y^2+(z-l1)^2-l2^2-l4^2-(l3+l5+l6)^2);
ct3a=ct3a/(2*l2*sqrt(l4^2+(l3+l5+l6)^2));

%there are two possible values for the robot configuration, elbow up and
%elbow down
t3_up=atan2(+sqrt(1-ct3a^2),ct3a)-a;
t3_down=atan2(-sqrt(1-ct3a^2),ct3a)-a;

% choosing elbow up configuration
t3=t3_up;
c3=cos(t3);
s3=sin(t3);

% defining ks and bs
k1=4*c3+20*s3;
k2=20*c3-4*s3+l1;
b1=-sqrt(x^2+y^2);
b2=z-l1;

% using matrix form
K=[k1,k2;k2,-k1];
B=[b1;b2];

trig2=inv(K)*B;
c2=trig2(1);
s2=trig2(2);
t2=atan2(s2,c2);

q=[t1,t2,t3,0,0];
q=double(q);


end

