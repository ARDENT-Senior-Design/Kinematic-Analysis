clc
clear all

x = [-0.5 -0.25 0 0.25 0.5]; y = [-x(1)^2 -x(2)^2 -x(3)^2 -x(4)^2 -x(5)^2]; z=[-x(1)^2 -x(2)^2 0 -x(4)^2 -x(5)^2]  %data
p1 = polyfit(x,y,4)   %get the polynomial
p2 = polyfit(x,z,4)
% Compute the values of the polyfit estimate over a finer range, 
% and plot the estimate over the real data values for comparison:
x2 = -0.5:0.1:0.5;          
y2 = polyval(p1,x2);
z2 = polyval(p2,x2);

plot3(x2,y2,z2)
xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
grid on
