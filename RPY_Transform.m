
%% Calcultate the transform from the center of the body to a leg
syms t_d t_y t_r t_p a_b0

t_d = 0.785;
a_b0 = 0.2
t_y = 0;
t_r = 0;
t_p = 0;
% Angles for displacement for the leg, yaw, roll, and pitch 
% a_b0: radial distance to leg

Tz = [cos(t_d+t_y) -sin(t_d+t_y) 0 0;
      sin(t_d+t_y) cos(t_d+t_y) 0 0;
      0 0 1 0;
      0 0 0 1];
  
Tx = [1 0 0 a_b0;
    0 cos(t_r) -sin(t_r) 0;
    0 sin(t_r) cos(t_r) 0;
    0 0 0 1];
Ty = [cos(t_p) 0 sin(t_p) 0
      0 1 0 0
      -sin(t_p) 0 cos(t_p) 0
      0 0 0 1];

T_b0 = Tx*Ty*Tz 

%% Calculate the position from the body origin to the foot

syms a_12 a_23 a_3e
a_12 = 0.1;
a_23 = 0.25;
a_3e = 0.25;

syms t b1 a 
% t: theta for the coxa relative to angle displacement from body
% b1: beta prime angle for the femur relative to coxa
% a: gamma for the angle of the tibia relative to the femur

% a_12;  femur link length
% a_23;  tibia link length
% a_3e: distance from tibia joint to end effector

t = 0;
b1 =0;
a=0;

T_01 = [cos(t) -sin(t) 0 0;
        sin(t) cos(t)  0 0;
        0       0      1 0;
        0       0      0 1];
    
T_12 = [cos(b1) -sin(b1) 0 a_12;
        0       0      -1 0;
        sin(b1) cos(b1) 0 0;
        0       0      0 1];
   
T_23 = [cos(a) -sin(a) 0 a_23;
        sin(a) cos(a)  0 0;
        0       0      1 0;
        0       0      0 1];
  

T_03 = T_01*T_12*T_23 % Transform from the body to the tiba joint



P_3e = [a_3e 0 0 1]';   % position of the end effector

P_be = T_b0*T_03*P_3e

%% Inverse Kinematics for an independent leg

%This calculator will take an end point and calculate the leg angles
%required to reach it relative to the coxa joint. This does not incorporate
%body orientation because I want to separate that calculation for cognition
x = 0.1;   
y = 0.0;
z = 0.2;

P_ee = [x, y, -z]'

inv_t = atan2(P_ee(2),P_ee(1))  % calculate the theta for the coxa

% rotate about the x-axis so we are in-line with the other leg axis (y is
% vertical)
P_ee = P_ee - [a_12 0 0]'; % move over to the femur joint

P_2e  = P_ee(1)^2 + P_ee(3)^2;   % calculate the hypotenuse^2 from femur joint to ee

alpha = atan2(-P_ee(3),P_ee(1))%-0.5*pi();   % calculate part of the b1 angle 

num = (a_23^2+P_2e-a_3e^2)/(2.0*a_23*sqrt(P_2e));
if num>1 % constrain 0->pi or 0->-pi
   num = 1;
end
if num<-1
    num=-1;
end

beta = acos(num);   % calculate the other part of the b1 angle

inv_b1 = beta+alpha;

num2 = (a_3e^2+a_23^2-P_2e)/(2.0*a_23*a_3e); % law of cosines for tibia angle

if num2>1   
   num2 = 1;
end
if num2<-1
    num2=-1;
end

inv_a = acos(num2) - pi();  % z- displaced by an additional +- 0.06

inv_t
inv_b1
inv_a

