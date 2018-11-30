% https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=743574
% assume robot platform is flat to the ground
m_coxa = 0.1;
w_coxa = m_coxa*9.8;
m_femur = 0.3;  
w_femur = m_femur*9.8;% Newtons
m_tibia = 0.3;
w_tibia = m_tibia*9.8;% Newtons
m_c = 15.9;
w_c = 156;  % Newtons
a_c1 = 0.2; % radius to coxa
a_12 = 0.15; % distance from femur to tibia
a_23 = 0.25;

[R1_01,R1_12,R1_23,R1_r] = FK(0,0,0)
[R2_01,R2_12,R2_23,R2_r] = FK(0,0,0)

R_c = [cos(0) 0 sin(0);
        sin(0) 0 -cos(0);
        0   0   1 ];
% %R radius, phi_z, th1_2, th1_3, th2_2, th2_3
% calculate the required torque for the centroid to stay up
dddot_Rx = 0;   % acceleration in x direction
dddot_Ry = 0;
Tx = m_c*dddot_Rx/2 %tensile forces acting on the centroid from legs
Ty = (m_c*dddot_Ry+w_c)/2
Tz = 0;

R_b = [a_c1 0 0];
M_c = cross((R_b-[0 0 0]),[Tx,Ty,0]') + cross((-R_b-[0 0 0]),[Tx,Ty,0]')

% femur leg 
dddot_Rax1 = 0;
dddot_Ray1 = 0;
T_jx1=m_femur*dddot_Rax1+Tx
T_jy1=m_femur*dddot_Ray1+Ty+w_femur
R_a = [a_12/2 0 0]
R_j = [a_12 0 0]
M_a = cross((R_b-R_a),-[Tx Ty 0]')+cross((R_j-R_a),[T_jx1 T_jy1 0]')
% wrong ( I think ), I need to double check

function [R_01,R_12,R_23,r] = FK(th_1, th_2, th_3)
g_off = deg2rad(60); % offset angle between legs
% th_1 = 0;   % coxa angle
% th_2 = 0;   % femur angle
% th_3 = 0;   % tibia angle
a_c1 = 0.2; % radius to coxa
a_12 = 0.15; % distance from femur to tibia
a_23 = 0.25;    % distance from tibia to ee
T_c0 = [-cos(g_off) sin(g_off) 0 a_c1*cos(g_off)
        0   0   1   0;
        sin(g_off) cos(g_off) 0 -a_c1*sin(g_off)
        0   0   0   1];
% coxa rotation
T_01 = [cos(th_1) 0 sin(th_1) 0;
        sin(th_1) 0 -cos(th_1) 0;
        0   1   0   0;
        0   0   0   1];
R_01 = T_01(1:3,1:3);
% femur rotation
T_12 = [cos(th_2) -sin(th_2) 0 a_12*cos(th_2);
        sin(th_2)   cos(th_2) 0 a_12*sin(th_2);
        0   0   1   0;
        0   0   0   1];
R_12 = T_12(1:3,1:3);
T_23 = [cos(th_3) -sin(th_3) 0 a_23*cos(th_3);
        sin(th_3)  cos(th_3) 0 a_23*sin(th_3);
        0   0   1   0;
        0   0   0   1];
R_23 = T_23(1:3,1:3);

r = T_01*T_12*T_23*([0,0,0,1]') % position of the end effector
end


