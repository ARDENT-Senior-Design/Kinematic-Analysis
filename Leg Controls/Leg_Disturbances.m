function F_eff = Leg_Disturbances(q_state)
q = q_state(1:3, 1)
q_dot = q_state(4:6, 1)
q_ddot = q_state(7:9,1)
%% 
motor_in_link = logical(cast(q_state(10),'uint8'));
operational_space = q_state(10);  %https://studywolf.wordpress.com/2013/09/17/robot-control-4-operation-space-control/
g = 9.8;

% Weight factor of safety
FoS = 1.25;

% legs
num_legs = 4;

% Masses of the robot and components
femur_motor_m = 0.7;    % Mass of the motor
femur_link_m = .05; % Mass of the link 
femur_gearbox_m = 0.85;  % Mass of the gearbox attached to motor 1:25
femur_joint_m = 0.6;   % Mass of the joint structure
m2_link = femur_motor_m+femur_link_m+femur_gearbox_m; % estimated mass with the motors in the middle of the leg
m2_joint = femur_motor_m+femur_gearbox_m+femur_joint_m;   % estimated mass with the motors in the joint


tibia_motor_m = 0.35;
tibia_link_m = .05;
tibia_gearbox_m = 0.6;
tibia_joint_m = 0.3;
m3_link = tibia_motor_m+tibia_link_m+tibia_gearbox_m;
m3_joint = femur_motor_m+femur_gearbox_m+tibia_joint_m;


battery_weight_increase = 18.5;
electronics_weight = 0+battery_weight_increase;
sensor_weight =2.3;
chassis_weight=2.0;
                                                                                                    % |
                                                                                                    %\l/-----m3_link is basically the coxa here
m = ((num_legs-3)*(m2_link+femur_joint_m+m3_link+tibia_joint_m+0.05)+m3_link*num_legs+sensor_weight+electronics_weight+chassis_weight)*FoS; %kg : mass of everything except 3 legs (excluding coxa)
m_total_no_FoS =  (m/FoS)+((num_legs-(num_legs-3))*(m2_link+femur_joint_m+m3_link+tibia_joint_m+0.05))
m_total = m+((num_legs-(num_legs-3))*(m2_link+femur_joint_m+m3_link+tibia_joint_m+0.05))*FoS
m1 = m/2; %For side with 1 leg down, it should take roughly half the weight of the robot

% Dimensions of leg
l1 = 0.4;   % width of the chassis (Ignore this value)
l2 = 0.25;  % length of the femur
l3 = 0.25;  % length of the tibia
r1 = l1/2;
r2 = l2/2;
r3 = l3/2;


% Angle of the joints
th1 = q(1); % angle of the body from the opposite leg. This would probably be at zero unless body is inclined. Doesn't really matter now.
th2 = q(2); % femur angle relative to body. 
                             % Peak Torque Combination Condition 1: th2 = 0
                             % Peak Torque Combination Condition 2: th2 = 30
th3 = q(3); % tibia angle relative to femur angle.  
                               %Peak Torque Combination Condition 1: th3 = 30
                               %Peak Torque Combination Condition 2: th3 = 0

N = m1*g;  % the force on the foot should be half the weight of the robot

% Angular velocities in rad/s.
% Keep these at 0 for standing
th_dot1 = q_dot(1);
th_dot2 = q_dot(2); %rad/s 
th_dot3 = q_dot(3);

% Accelerations of the joints rad/s^2
th_ddot1 = q_ddot(1);
th_ddot2 = q_ddot(2);   % accelerate to 0.025 rad/s2 in 1/200th of second
th_ddot3 = q_ddot(3);

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

    I(1,1) =0; % we don't need the data for the chassis
    I(1,2) = 0.01;
    I(1,3) = i3+m3_link*r3^2;

    I(2,1) =I(1,2);
    I(2,2) = i2+i3+m2_link*r2^2+m3_link*(l2^2+r3^2+2*l2*r3*cos(th3));
    I(2,3) = i3+m3_link*(r3^2+l2*r3*cos(th3));

    I(3,1) = 0;
    I(3,2) = I(2,3);
    I(3,3) = i3+m3_link*r3^2
else  
    I(1,1) =0; % we don't need the data for the chassis
    I(1,2) = 0.01;
    I(1,3) = 0.01;

    I(2,1) =I(1,2);
    I(2,2) = i2+i3+femur_link_m*r2^2+m3_link*(l2^2+r3^2+2*l2*r3*cos(th3));
    I(2,3) = i3+m3_link*(r3^2+l2*r3*cos(th3));

    I(3,1) = 0;
    I(3,2) = I(2,3);
    I(3,3) = i3+tibia_link_m*r3^2;
end
I
th = [th1, th2, th3];
Jaco = jacobian(th)

x = Jaco'*[0,0,N,0,0,0]'    % apply a force load in the vertical z-direction 
if(operational_space == false)
    F_eff = I*[th_ddot1; th_ddot2; th_ddot3]
else
    det(I)
    M = Jaco'\(I*pinv(Jaco))
    % M = (Jaco*(Jaco'\I))^(-1)
    F_eff = Jaco'*M*[th_ddot1; th_ddot2; th_ddot3]
end