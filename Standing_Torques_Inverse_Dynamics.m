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
m = 3*(m2+m3+0.05)+0.6*6+2.3+5.6+1.68 %kg : mass of everything except 3 legs (excluding coxa)
m_total = m+3*(m2+m3+0.05)
m1 = m/2; %For side with 1 leg down, it should take roughly half the weight of the robot

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

% G(1) = -g*((N*l1/g+m3*l1+m2*l1+m1*r1)*cos(th1) + (N*l2/g+m3*l2+m2*r2)*cos(th1+th2) + (N*l3/g+m3*r3)*cos(th1+th2+th3)); 
% G(2) = -g*((N*l2/g+m3*l2+m2*r2)*cos(th1+th2) + (N*l3/g+m3*r3)*cos(th1+th2+th3));
% G(3) = -g*((N*l3/g+m3*r3)*cos(th1+th2+th3));
% G = G'

G(1) =g*((m3*l1+m2*l1+m1*r1)*cos(th1) + (m3*l2+m2*r2)*cos(th1+th2) + (N*l3/g+m3*r3)*cos(th1+th2+th3)); 
G(2) =g*((m3*l2+m2*r2)*cos(th1+th2) + (m3*r3)*cos(th1+th2+th3));
G(3) =g*((m3*r3)*cos(th1+th2+th3));
G = G

th_dot1 = 0;
th_dot2 = 6.25; %rad/s roughly 60rpm
th_dot3 = 6.25;

V(1) = (th_dot2^2)*(m2*l1*r2*sin(th2))+(th_dot1*th_dot2)*(2*m2*l1*r2*sin(th2)); % might be slightly wrong
V(2) = (th_dot3^2)*(m3*l2*r3*sin(th3))+(th_dot3*th_dot2)*(2*m3*l2*r3*sin(th3));
V(3) = (th_dot2^2)*(m3*l2*r3*sin(th3));
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


% calculate the jacobian for the external force acting on it
T1=0;T2=45;T3=45; %angles of joints
a1=.1;a2=.15;a3=.25; %link lengths

H0_1 = [cos(T1) -sin(T1) 0 a1;
        sin(T1) cos(T1)  0 0;
        0       0      1 0;
        0       0      0 1];
    
H1_2 = [cos(T2) -sin(T2) 0 a2;
        0       0      -1 0;
        sin(T2) cos(T2) 0 0;
        0       0      0 1];
   
H2_3 = [cos(T3) -sin(T3) 0 a3;
        sin(T3) cos(T3)  0 0;
        0       0      1 0;
        0       0      0 1];
R0_0 = [1 0 0; 0 1 0; 0 0 1];
R0_1 = H0_1(1:3,1:3);
R1_2 = H1_2(1:3,1:3);
H0_2=H0_1*H1_2 %coxa to tibia
R0_2=H0_2(1:3,1:3);
H0_3=H0_2*H2_3 %coxa to end effector
R0_3=H0_3(1:3,1:3);


k=[0;0;1]; %k constant
k0_0=R0_0*k; %Z column for rotation matrix at the origin X's k-constant
k0_1=R0_1*k; %Z column for rotation matrix from coxa to femur X's k-constant
k0_2=R0_2*k;%Z column for rotation matrix from coxa to tibia X's k-constant

dz1=H0_3(1:3,4); %displacment from the homogenous matrix for coxa to end effector
dz2=H0_3(1:3,4)-H0_1(1:3,4);%displacement for coxa to end effector minus coxa to femur
dz3=H0_3(1:3,4)-H0_2(1:3,4);%displacement for coxa to end effector minus coxa to tibia

jx=cross(k0_0,dz1); %cross product between Z axis rotation and displacement z1
jy=cross(k0_1,dz2); %cross product between Z axis rotation and displacement z2
jz=cross(k0_2,dz3); %cross product between Z axis rotation and displacement z3

jwx=(k0_0); %jacobian angular vel x-axis (rotation X's k-constant 
jwy=(k0_1); %jacobian angular vel y-axis (rotation X's k-constant
jwz=(k0_2); %jacobian angular vel z-axis (rotation X's k-constant


Jv=[jx jy jz] %jacobian linear matrix. Wrong, it should include a jx component based on the sum of the two angles
Jw=[jwx jwy jwz] %jacobian angular matirx

Jaco=[Jv; Jw]
Jaco'
%eev=[xdot; ydot; zdot]
%eew=[wx; wy; wz]
% vel = Jaco*eev

x = Jaco'*[0,N,0,0,0,0]'

T = I*[th_ddot1; th_ddot2; th_ddot3]+(V+G')
% numbers roughly check out. %T(1) is basically useless and there to prove
% body torques

%% Torque required to walk forward at varying inclines
% T=T1-F1(L1cos(th1)+L2cos(th2))-L3Wx+2*F2*(2*L3+L1*cos(th1)+L2*cos(th2))
% Calculates the torque required to stay stationary on an incline, doesn't
% include dynamics
us = 0.5;   % Coefficient of friction
a = 5;
B = deg2rad(45); %incline of slope
F_f =us*N*cos(B)   % force required due to friction
F(2) = F_f/2;
F(1) = F(2)*2; 
T1 = F(1)*(l3*cos(th1+th2+th3)+l2*cos(th1+th2))+(l1/2)*(m*g*sin(B)) - 2*F(2)*(l1+l3*cos(th1+th2+th3)+l2*cos(th1+th2))+m*a*l3/2
   
if T1 < 0
    T1 = m*a*l3/2
    T2 = m*a*l3/2
else
    T2 =  2*F(2)*(l3*cos(th1+th2+th3)+l2*cos(th1+th2))+(l1/2)*(m*g*sin(B)) -F(1)*(l1+l3*cos(th1+th2+th3)+l2*cos(th1+th2)) +m*a*l3/2
    T2_n = T2/2
end
% hello there

