clc;
clear all;

%     (2)  O
%            |   \
%            |     O (1)
%            |   /
%     (3) O
%

c = [0,0];
walking_height = 0.2;
r = 0.2;
ang_disp = [0 120 240];
x_disp = (0.25+r).*cosd(ang_disp(:));
y_disp = (0.25+r).*sind(ang_disp(:));

for i=1:3
    p(i,:) = [x_disp(i),y_disp(i)]
end
% Hard code values
% p(1,:) = [0.35,-0.35];
% p(2,:) = [0.25,0.25];
% p(3,:) = [-0.35,-0.35];
slope_angle = 50;
for i=1:3
    p(i,:) = [p(i,1),p(i,2)*cosd(slope_angle)]
end
c(1,2)
c(1,2) = c(1,2)-walking_height*sind(slope_angle)

S(1) = 0.5*det([1 1 1; c(1) p(1,1) p(2,1); c(2) p(1,2) p(2,2)]);
S(2) = 0.5*det([1 1 1; c(1) p(2,1) p(3,1); c(2) p(2,2) p(3,2)]);
S(3) = 0.5*det([1 1 1; c(1) p(3,1) p(1,1); c(2) p(3,2) p(1,2)]);
S