clc;
clear all;

gait = 'creep'; %[ crawl, walking_trot, running_trot, bound, rotary_gallop]
leg_max_r = 0.25+0.25*cosd(30);
L = leg_max_r/2 %stride length [m]
com_x = 0; com_y = 0; % center of mass position [m]
com_vx = 0.5; com_vy = 0; % center of mass velocity [m/s]
T = 0;  % Period of one cycle.
if(strcmp(gait,'creep'))
    T = (2*L)/com_vx   % a leg enters swing state every half stride length. 
    % time it takes to do 1 oscillation (left+right move)
end

subplot(3,1,1) 
grid on;
a1 = animatedline('Color',[0 .7 .7]);
t = linspace(0,10,1000);
axis([0 10 -5 5])
ylabel('CoM X-Position [m]');
xlabel('Time [s]');
for ti = 1:length(t);
    % 3 figures. Top one will be the trajectory in xy. 
    % Middle will be the "PWM" % gait layout
    % CoM position
    % leg 2,1,4,3 https://link.springer.com/article/10.1007/s11370-015-0173-2
    %crawl can transition
    % first line
    com_x = com_vx * t(ti);
    addpoints(a1,t(ti),com_x);
    % update screen
    drawnow 
end

subplot(3,1,2) 
grid on;
a2 = animatedline('Color',[0 .7 .7]);
t = linspace(0,20,1000);
axis([0 10 -0.25 0.25])
ylabel('CoM Y-Position [m]');
xlabel('CoM X-Position [m]]');
for ti = 1:length(t);
    % 3 figures. Top one will be the trajectory in xy. 
    % Middle will be the "PWM" % gait layout
    % CoM position
    % leg 2,1,4,3 https://link.springer.com/article/10.1007/s11370-015-0173-2
    %crawl can transition
    % first line
    com_x = com_vx * t(ti);
    com_y = sin(com_x)*0.125;
    addpoints(a2,com_x,com_y);
    % update screen
    drawnow 
end