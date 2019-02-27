clc; clear;
T1=0;T2=0;T3=0; %angles of joints
% syms T1 T2 T3
xdot=0;ydot=0;zdot=10.0; %end effector linear velocity
wx=0;wy=0;wz=0; %angular end effector velocity
w_joints = [ 0; 0; 0]

k=[0;0;1]; %k constant
a1=.1;a2=.25;a3=.25; %link lengths
%syms a1 a2 a3
H0_1 = [cosd(T1) 0 sind(T1) a1*cosd(T1);
        sind(T1) 0  -cos(T1) a1*sind(T1);
        0       1     0 0;
        0       0      0 1];
    
H1_2 = [cosd(T2) -sind(T2) 0 a2*cosd(T2);
        sind(T2)       cosd(T2)      0 a2*sind(T2);
        0       0      1 0;
        0       0      0 1];
   
H2_3 = [cosd(T3) -sind(T3) 0 a3*cosd(T3);
        sind(T3)       cosd(T3)      0 a3*sind(T3);
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

dz1=H0_3(1:3,4) %displacment from the homogenous matrix for coxa to end effector
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
eev=[xdot; ydot; zdot; wx; wy; wz];
vel = Jaco*w_joints  % the velocitites are based on the axis of the base origin

w_joint_ik = (Jaco)\eev %joint velocity matrix = inverse jacobian X's end effector linear velocity matrix J^(-1)*ee_dot = q_dot
%Tdotw=inv(Jw) %joint velocity matrix = inverse jacobian X's end effector angular velocity matrix

