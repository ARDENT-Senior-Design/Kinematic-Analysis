clc;
clear all;
%% Robot Parameters
m1 = 0.1;
m2 = 0.1;
D = 0.01;
C_c = 0.27;     % Constant for coulomb friction
C_v = 0.01;      % Constant for coulomb friction
a = 100;            % Constant 
T = 0;                % Friction 
u = 0;                % Additional torque input for identification
p = [T]';             % Unknown backlash and spring function to estimate
Kr = 0;               % P controller for position
dt = 0.1;
%% Augmented State 
%[x_dot; p_dot] = [f(x,u,p); 0]+v

v = [0 1 2 3 4];    % process noise with 0 mean
x3 = 0;             % velocity of the first link
x4 = 0;             % velocity of the second link
x1_dot = x3 + v(1);    
x2_dot = x4 + v(2);
x3_dot = (1/m1)*(u-D*(x3-x4)-p-T-Kr) + v(3);
x4_dot = (1/m2)*(D*(x3-x4)+p)+v(4);
p_dot = v(5);

%% Covariance for Augmented noise process v
Q_xx = [0,0 ; 0,0];    % Covariance of the noise in the state derivatives
Q_pp = 0;               % Covariance of the noise on the augmented state derivative
                                % if Q_pp is 0, p will remain constant. If Q_pp increases, the augmented state update rate increases
E = [Q_xx 0; 0 Q_pp]*dirac(dt)