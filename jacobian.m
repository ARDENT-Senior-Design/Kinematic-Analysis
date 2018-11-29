clc; clear;
T1=pi/2;T2=pi/3;T3=pi/4; %angles of joints
xdot=1;ydot=1;zdot=1; %end effector linear velocity
wx=1;wy=1;wz=-0.75; %angular end effector velocity
k=[0;0;1]; %k constant
a1=.25;a2=.25;a3=.25; %link lengths

R0_0=[1 0 0;0 1 0;0 0 1]; %rotation at the origin [coxa]
R0_1=[cos(T1) -sin(T1) 0; 0 0 1; -sin(T1) -cos(T1) 0]; %rotation coxa to femur joint 
R1_2=[cos(T2) -sin(T2) 0;sin(T2) cos(T2) 0; 0 0 1]; %rotation femur to tibia
R2_3=[cos(T3) -sin(T3) 0; 0 0 -1; sin(T3) cos(T3) 0]; %rotation tibia to end effector
R0_2=[cos(T1+T2) -sin(T1+T2) 0; 0 0 1; -sin(T1+T2) -cos(T1+T2) 0]; %rotation from coxa to femur [R0_1*R1_2]

D0_1=[a1*cos(T1);a1*sin(T1);0]; %displacement coxa to femur
D1_2=[a2*cos(T2);a2*sin(T2);0]; %femur to tibia
D2_3=[a3*cos(T3);a3*sin(T3);0]; %tibia to end effector

H1_2=[R1_2 D1_2;0 0 0 1]; %homogenous transformation from femur to tibia
H2_3=[R2_3 D2_3;0 0 0 1]; %femur to end effector

H0_1=[R0_1 D0_1;0 0 0 1]; %coxa to femur
H0_2=H0_1*H1_2; %coxa to tibia
H0_3=H0_2*H2_3; %coxa to end effector

k0_0=R0_0(:,3).*k; %Z column for rotation matrix at the origin X's k-constant
k0_1=R0_1(:,3).*k; %Z column for rotation matrix from coxa to femur X's k-constant
k0_2=R0_2(:,3).*k;%Z column for rotation matrix from coxa to tibia X's k-constant

dz1=H0_3(1:3,4); %displacment from the homogenous matrix for coxa to end effector
dz2=H0_3(1:3,4)-H0_1(1:3,4);%displacement for coxa to end effector minus coxa to femur
dz3=H0_3(1:3,4)-H0_2(1:3,4);%displacement for coxa to end effector minus coxa to tibia

jx=cross(k0_0,dz1); %cross product between Z axis rotation and displacement z1
jy=cross(k0_1,dz2); %cross product between Z axis rotation and displacement z2
jz=cross(k0_2,dz3); %cross product between Z axis rotation and displacement z3

jwx=(k0_0); %jacobian angular vel x-axis (rotation X's k-constant 
jwy=(k0_1); %jacobian angular vel y-axis (rotation X's k-constant
jwz=(k0_2); %jacobian angular vel z-axis (rotation X's k-constant


Jv=[jx jy jz] %jacobian linear matrix
Jw=[jwx jwy jwz] %jacobian angular matirx

Jaco=[Jv;.6-----------------------=--===========-===========-==-==-=-=--=---------7745 Jw]
%eev=[xdot; ydot; zdot]
%eew=[wx; wy; wz]

%Tdotv=inv(Jv) %joint velocity matrix = inverse jacobian X's end effector linear velocity matrix
%Tdotw=inv(Jw) %joint velocity matrix = inverse jacobian X's end effector angular velocity matrix

