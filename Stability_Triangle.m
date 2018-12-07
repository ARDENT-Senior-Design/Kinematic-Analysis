clc;
clear all;

c = [0,0];

r = 0.2;
ang_disp = [0 120 240];
x_disp = (0.25+r).*cosd(ang_disp(:));
y_disp = (0.25+r).*sind(ang_disp(:));

for i=1:3
    p(i,:) = [x_disp(i),y_disp(i)]
end
p(3,:) = [-0.45,0]
S(1) = 0.5*det([1 1 1; c(1) p(1,1) p(2,1); c(2) p(1,2) p(2,2)]);
S(2) = 0.5*det([1 1 1; c(1) p(2,1) p(3,1); c(2) p(2,2) p(3,2)]);
S(3) = 0.5*det([1 1 1; c(1) p(3,1) p(1,1); c(2) p(3,2) p(1,2)]);
S