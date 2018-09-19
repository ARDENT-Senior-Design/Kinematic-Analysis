hold on
l1 = 10; % length of first arm
l2 = 7; % length of second arm
l3 = 10;
theta1 = 0:0.1:pi/2; % all possible theta1 values
theta2 = 0:0.1:pi; % all possible theta2 values
theta3 = 0:0.1:pi/2;
[THETA1,THETA2,THETA3] = meshgrid(theta1,theta2,theta3); % generate a grid of theta1 and theta2 values

X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % compute x coordinates
Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % compute y coordinates
Z = l1 + l2*sin(THETA2)-l3*sin(THETA3+THETA2);
data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-theta2 dataset
data3 = [X(:) Y(:) THETA3(:)];

 plot3(X(:),Y(:),Z(:),'r.'); 
   xlabel('X','fontsize',10)
  ylabel('Y','fontsize',10)
  zlabel('Z','fontsize',10)
  
  %Link Lengths
	a2=10;
	a4=7;
	%Desired X-Y-Position of the end effector
	X=10;
	Y=7;
    t=1;
    %Desired X-Y-Position of the end effector as a function of time
	xt=X*t; 
	yt=Y*t; 
	%Equations used to calculate theta 1 and theta 2
	r1=sqrt(xt.^2+yt.^2);
	phi1=acos(((a4.^2)-(a2.^2)-(r1.^2))/(-2*a2*r1));
	phi2=atan2(yt,xt);
	phi3=acos(((r1.^2)-(a2.^2)-(a4.^2))/(-2*a2*a4));
	%Theta 1 in radians
	T1=phi2-phi1;
	%Theta 2 is (180 degrees - angle phi3)
	T2=pi-phi3;
	%Vector coordinates of the linkage
	x=[0 a2*cos(T1) xt];
	y=[0 a2*sin(T1) yt];
    z=[0 7 10];
	%Theta in Degrees
	T1=T1*(180/pi)
	T2=T2*(180/pi)
	%Plot end effector 
   	plot3(x,y,z,'o-'); 
    axis equal
	hold on
  