clc;
clear all;
%% Full body Inverse Dynamics of a standing hexapod robot with 3 legs down
%      ...o------o----C----o  
%    th3 /      th2       th1
%       /
%      /
%
% SI units
g = 9.8;

m2 = 0.8; % estimated mass with the motors in the middle of the leg
m3 = 0.8;
m = 0 %kg : mass of everything except 3 legs (excluding coxa)
m1 = 0; %For side with 1 leg down, it should take roughly half the weight of the robot

l1 = 0.4;   % width of the chassis
l2 = 0.15;  % length of the femur
l3 = 0.25;  % length of the tibia
r1 = l1/2;
r2 = l2/2;
r3 = l3/2;

th1 = deg2rad(0); % angle of the body from the opposite leg. This would probably be at zero unless inclined
th2 = deg2rad(0); % femur angle relative to body
th3 = deg2rad(0); % tibia angle relative to femur angle

N = m1*g;  % the force on the foot should be half the weight of the robot

G(1) = -g*((N*l1/g+m3*l1+m2*l1+m1*r1)*cos(th1) + (N*l2/g+m3*l2+m2*r2)*cos(th1+th2) + (N*l3/g+m3*r3)*cos(th1+th2+th3)); 
G(2) = -g*((N*l2/g+m3*l2+m2*r2)*cos(th1+th2) + (N*l3/g+m3*r3)*cos(th1+th2+th3));
G(3) = -g*((N*l3/g+m3*r3)*cos(th1+th2+th2));
G = G'

th_dot1 = 0;
th_dot2 = 6.25; %rad/s roughly 60rpm
th_dot3 = 6.25;

V(1) = (th_dot2^2)*(m2*l1*r2*sin(th2))+(th_dot1*th_dot2)*(2*m2*l1*r2*sin(th2)); % might be slightly wrong
V(2) = (th_dot3^2)*(m3*l2*r3*sin(th3))+(th_dot3*th_dot2)*(2*m3*l2*r3*sin(th3));
V(3) = -(th_dot2^2)*(m3*l2*r3*sin(th3));
V = V'

% Inertias are about the CoG for 1,2,3
i1 = 0.76
i2 = 0.0013;    
i3 = 0.0036;

I(1,1) = i1+i2+i3+m1*r1^2+m2*(l1^2+r2^2+2*l1*r2*cos(th2))+m3*(l1^2+r3^2+2*l2*r3*cos(th3)); % lengths might be slightly wrong
I(1,2) = i2+i3+m2*r2^2+m3*(l1^2+r3^2+l2*r3*cos(th2));
I(1,3) = i2+i3+m3*r3^2+m3*l2*r3*cos(th2);

I(2,1) = I(1,2);
I(2,2) = i2+i3+m2*r2^2+m3*(r3^2+l2^2+2*r3*l2*cos(th3));
I(2,3) = i3+m3*(r3^2+l2*r3*cos(th3));

I(3,1) = I(1,3);
I(3,2) = I(2,3);
I(3,3) = i3+m3*r3^2

th_ddot1 = 0;
th_ddot2 = 20;   % accelerate to 0.025 rad/s2 in 1/200th of second
th_ddot3 = 20;
T = I*[th_ddot1; th_ddot2; th_ddot3]-(V+G)
% numbers roughly check out. %T(1) is basically useless and there to prove
% body torques