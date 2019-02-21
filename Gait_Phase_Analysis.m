clc;
clear all;
a1 = animatedline('Color',[0 .7 .7]);

gait = 'creep'; %[ crawl, walking_trot, running_trot, bound, rotary_gallop]
stride_length = 0.25 %[m]
com_x = 0; com_y = 0;
[com_x,com_y];

axis([0 20 -1 1])
x = linspace(0,20,1000);
for k = 1:length(x);
    % 3 figures. Top one will be the trajectory in xy. 
    % Middle will be the "PWM" % gait layout
    % CoM position
    % leg 2,1,4,3 https://link.springer.com/article/10.1007/s11370-015-0173-2
    %crawl can transition
    
    % first line
    com_x = x(k);
    com_y = sin(com_x);
    addpoints(a1,com_x,com_y);


    % update screen
    drawnow 
end