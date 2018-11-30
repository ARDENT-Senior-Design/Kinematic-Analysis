clc;
clear all;

%% Full body Inverse Dynamics of a standing hexapod robot with 3 legs down
% SI units
g = 9.8;
m = 18; %kg
m1 = m/2; %For side with 1 leg down, it should take roughly half the weight of the robot
m2 = 0.6; % estimated mass with the motors in the middle of the leg
m3 = 0.6;

l1 = 0.2;
l2 = 0.15;
l3 = 0.25;
r1 = l1/2;
r2 = l2/2;
r3 = l3/2;

th1 = deg2rad(0); % angle of the body from the opposite leg
th2 = deg2rad(0);
th3 = deg2rad(-60);

N = 0.5*m*9.8;  % the force on the foot should be half the weight of the robot

g(1) = -g*((N*l1/g+m3*l1+m2*l1-m1*r1)*cos(th1) + (N*l2/g+m3*l2+m2*r2)*cos(th1+th2) + (N*l3/g+m3*r3)*cos(th1+th2+th3)) 
g(2) = -g*((N*l2/g+m3*l2+m2*r2)*cos(th2) + (N*l3/g+m3*r3))