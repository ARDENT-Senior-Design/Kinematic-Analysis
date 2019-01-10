clc;
clear all;

%% Full body Inverse Dynamics of a standing hexapod robot with 3 legs down
%      
%
%                      ---o----l1----o---------l2------o--------------o
%                     (CoM)          th1 (+CW)       th2(+Cw)
% 
%   Joint angles are relative to each other
%
% SI units
% 
%
%   THIS CALCULATOR ASSUMES THE COXA LENGTH IS 0 FOR SIMPLICITY
%
motor_in_link = false;


g = 9.8;

% Weight factor of safety
FoS = 1.25;

% Masses of the robot and components
femur_motor_m = 0.7;    % Mass of the motor
femur_link_m = .1; % Mass of the link 
femur_gearbox_m = 0.8;  % Mass of the gearbox attached to motor
femur_joint_m = 0.15;   % Mass of the joint structure
m2_link = femur_motor_m+femur_link_m+femur_gearbox_m; % estimated mass with the motors in the middle of the leg
m2_joint = femur_motor_m+femur_gearbox_m+femur_joint_m;   % estimated mass with the motors in the joint


tibia_motor_m = 0.325;
tibia_link_m = .1;
tibia_gearbox_m = 0.7;
tibia_joint_m = 0.15;
m3_link = tibia_motor_m+tibia_link_m+tibia_gearbox_m;
m3_joint = femur_motor_m+femur_gearbox_m+tibia_joint_m;


battery_weight_increase = 3;
electronics_weight = 4+battery_weight_increase;
sensor_weight =2.3;
chassis_weight=2;
                                                                                                    % |
                                                                                                    %\l/-----m3_link is basically the coxa here
m = (3*(m2_link+femur_joint_m+m3_link+tibia_joint_m+0.05)+m3_link*6+sensor_weight+electronics_weight+chassis_weight)*FoS; %kg : mass of everything except 3 legs (excluding coxa)
m_total_no_FoS =  (m/FoS)+(3*(m2_link+femur_joint_m+m3_link+tibia_joint_m+0.05))
m_total = m+(3*(m2_link+femur_joint_m+m3_link+tibia_joint_m+0.05))*FoS
m1 = m/2; %For side with 1 leg down, it should take roughly half the weight of the robot

% Dimensions of leg
l1 = 0.4;   % width of the chassis (Ignore this value)
l2 = 0.2;  % length of the femur
l3 = 0.25;  % length of the tibia
r1 = l1/2;
r2 = l2/2;
r3 = l3/2;

% Angle of the joints
th1 = deg2rad(0); % angle of the body from the opposite leg. This would probably be at zero unless body is inclined. Doesn't really matter now.
th2 = deg2rad(0); % femur angle relative to body. 
                             % Peak Torque Combination Condition 1: th2 = 0
                             % Peak Torque Combination Condition 2: th2 = -30
th3 = deg2rad(-30); % tibia angle relative to femur angle.  
                               %Peak Torque Combination Condition 1: th3 = -30
                               %Peak Torque Combination Condition 2: th3 = 0

N = m1*g;  % the force on the foot should be half the weight of the robot

% Angular velocities in rad/s.
th_dot1 = 0;
th_dot2 = 0; %rad/s roughly 60rpm
th_dot3 = 0;

% Accelerations of the joints rad/s^2
th_ddot1 = 0;
th_ddot2 = -40;   % accelerate to 0.025 rad/s2 in 1/200th of second
th_ddot3 = -40;

% Inertias are about the CoG for 1,2,3
i1 = 0 % this doesn't matter anymore
if motor_in_link == true
    i2 = (1/12)*m2_link*l2^2;    
    i3 =  (1/12)*m3_link*l2^2;
else
    i2 = (1/12)*femur_link_m*l2^2;    
    i3 =  (1/12)*tibia_link_m*l2^2;
end

% see: http://www-lar.deis.unibo.it/people/cmelchiorri/Files_Robotica/FIR_05_Dynamics.pdf

if motor_in_link == true
  
    G(1) = 0;% we don't need the data for the chassis
    G(2) =g*((m3_link*l2+m2_link*r2)*cos(th1+th2) + (m3_link*r3)*cos(th1+th2+th3));
    G(3) =g*((m3_link*r3)*cos(th1+th2+th3));
    G = G'

    V(1) = 0; % we don't need the data for the chassis
    V(2) = -m3_link*l2*r3*sin(th3)*(th_dot2*th_dot3+th_dot3^2);
    V(3) = -m3_link*l2*r3*sin(th3)*(-(th_dot2^2));
    V = V'

    I(1,1) =0; % we don't need the data for the chassis
    I(1,2) = 0;
    I(1,3) = 0;

    I(2,1) =0;
    I(2,2) = i2+i3+m2_link*r2^2+m3_link*(l2^2+r3^2+2*l2*r3*cos(th3));
    I(2,3) = i3+m3_link*(r3^2+l2*r3*cos(th3));

    I(3,1) = 0;
    I(3,2) = I(2,3);
    I(3,3) = i3+m3_link*r3^2

else
    
    % G(q)
    G(1) =0;
    G(2) =g*((m3_link*l2+femur_link_m*r2)*cos(th1+th2) + (m3_link*r3)*cos(th1+th2+th3));
    G(3) =g*((tibia_link_m*r3)*cos(th1+th2+th3));
    G = G'

    % V(q,q_dot)q_dot
    V(1) = 0; % we don't need the data for the chassis
    V(2) = -m3_link*l2*r3*sin(th3)*(th_dot2*th_dot3+th_dot3^2);
    V(3) = -m3_link*l2*r3*sin(th3)*(-th_dot2^2);
    V = V'

    I(1,1) =0; % we don't need the data for the chassis
    I(1,2) = 0;
    I(1,3) = 0;

    I(2,1) =0;
    I(2,2) = i2+i3+femur_link_m*r2^2+m3_link*(l2^2+r3^2+2*l2*r3*cos(th3));
    I(2,3) = i3+m3_link*(r3^2+l2*r3*cos(th3));

    I(3,1) = 0;
    I(3,2) = I(2,3);
    I(3,3) = i3+tibia_link_m*r3^2

end

% calculate the jacobian for the external force acting on it
T1=0;T2=th2;T3=th3; %angles of joints
% syms T1 T2 T3
xdot=0;ydot=1;zdot=0; %end effector linear velocity
wx=0;wy=0;wz=0; %angular end effector velocity
w_joints = [ 0; 1; 1]

k=[0;0;1]; %k constant
a1=0;a2=.2;a3=.25; %link lengths
%syms a1 a2 a3
H0_1 = [cos(T1) 0 sin(T1) a1*cos(T1);
        sin(T1) 0  -cos(T1) a1*sin(T1);
        0       1     0 0;
        0       0      0 1];
    
H1_2 = [cos(T2) -sin(T2) 0 a2*cos(T2);
        sin(T2)       cos(T2)      0 a2*sin(T2);
        0       0      1 0;
        0       0      0 1];
   
H2_3 = [cos(T3) -sin(T3) 0 a3*cos(T3);
        sin(T3)       cos(T3)      0 a3*sin(T3);
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
k0_0=R0_0*k %Z column for rotation matrix at the origin X's k-constant
k0_1=R0_1*k %Z column for rotation matrix from coxa to femur X's k-constant
k0_2=R0_2*k%Z column for rotation matrix from coxa to tibia X's k-constant

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

x = Jaco'*[0,0,N,0,0,0]'    % apply a force load in the vertical z-direction 


D = diag([0.0, 0.01, 0.01])    % friction
T = x-(I*[th_ddot1; th_ddot2; th_ddot3]+V+G+D*[th_dot1; th_dot2; th_dot3])

%% Torque required to walk forward at varying inclines
% T=T1-F1(L1cos(th1)+L2cos(th2))-L3Wx+2*F2*(2*L3+L1*cos(th1)+L2*cos(th2))
% Calculates the torque required to stay stationary on an incline, doesn't
% include dynamics
us = 0.5;   % Coefficient of friction
a = 5;  % Forward acceleration of the robot
B = deg2rad(40); %incline of slope
F_f =us*N*cos(B)   % force required due to friction
F(2) = F_f/2;
F(1) = F(2)*2; 
T1 = F(1)*(l3*cos(th1+th2+th3)+l2*cos(th1+th2))+(l1/2)*(m*g*sin(B)) - 2*F(2)*(l1+l3*cos(th1+th2+th3)+l2*cos(th1+th2))+m*a*l3/2;
   
if T1 < 0
    T1 = m*a*l3/2
    T2 = m*a*l3/2
else
    T2 =  2*F(2)*(l3*cos(th1+th2+th3)+l2*cos(th1+th2))+(l1/2)*(m*g*sin(B)) -F(1)*(l1+l3*cos(th1+th2+th3)+l2*cos(th1+th2)) +m*a*l3/2
    T2_n = T2/2;
end
% hello there

