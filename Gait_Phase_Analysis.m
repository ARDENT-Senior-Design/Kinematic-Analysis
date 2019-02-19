set(0,'defaultFigureColor',[1 1 1]);
clear all; clc;

addpath([pwd 'KinematicAnalysis/ds2nfu']);

isVideOn = 0;      % To control whether to dump jpeg snapshots or not. 0: do not dump 1: dump jpegs

Np = 220;            % Number of spatial points
dx = 2*pi/Np;        % discretization size
x  = 0:dx:(4*pi);    % utilized x range

% Individual Harmonics
A=4;
f1sin = A*sin(x)/pi;        f1cos = A*cos(x)/pi;
f2sin = A*sin(x-3.14)/(pi);  f2cos = A*cos(x-3.14)/(pi);
f1sin_c = f1sin; f1cos_c = f1cos;
f2sin_c = f2sin; f2cos_c = f2cos;
f3sin = A*sin(x-6.28)/pi;        f3cos = A*cos(x-6.28)/pi;
f4sin = A*sin(x-9.42)/(pi);  f4cos = A*cos(x-9.42)/(pi);
f3sin_c = f3sin; f3cos_c =  f3cos;
f4sin_c = f4sin; f4cos_c =  f4cos;
for i=1:length(x)
    if(x(i)-3.14 > 0)
        f1sin(i) = 0;
    end
    if(x(i)-6.28 > 0 || x(i)-3.14 < 0)
        f2sin(i) = 0;
    end
    if(x(i)-9.42 > 0 || x(i)-6.28 < 0)
        f3sin(i) = 0;
    end
    if(x(i)-12.56 > 0 || x(i)-9.42 < 0)
        f4sin(i) = 0;
    end
end
% f3sin = A*sin(5*x)/(5*pi);  f3cos = A*cos(5*x)/(5*pi);
% f4sin = A*sin(7*x)/(7*pi);  f4cos = A*cos(7*x)/(7*pi);
% f5sin = A*sin(9*x)/(9*pi);  f5cos = A*cos(9*x)/(9*pi);

% Use the following external package at Matlab Central:
% f=f1sin+f2sin;%+f3sin+f4sin+f5sin;

Nt = 100;                % Number of points over the circles
dtheta = 2*pi/Nt;        % Discretization over circles
theta  = 0:dtheta:2*pi;  % Span the whole 2pi

% Circle points
x1=A/pi*cos(theta);
y1=A/pi*sin(theta);

x2=A/(pi)*cos(theta);
y2=A/(pi)*sin(theta);

x3=A/pi*cos(theta);
y3=A/pi*sin(theta);

x4=A/(pi)*cos(theta);
y4=A/(pi)*sin(theta);

Lx=length(x);
Lw=2; Fs=12;
for i=1:length(x)

    %disp([num2str(i) ' of ' num2str(Lx)])
    f1=figure (2); clf;

    sp1=subplot(1,2,1);
    %  -- 1st harmonic ---
    plot(x1,y1,'LineWidth',Lw,'Color','b'); hold on; grid on;
    line([0 f1cos_c(i)],[0 f1sin_c(i)],'Color','b','LineWidth',Lw,'LineSmoothing','on');

    set(sp1,'Position',[0.0400    0.1800    0.4    0.677]);
    xlim([-2.5 2.5]); ylim([-2.5 2.5])

    [xf1, yf1] = ds2nfu(sp1,f1cos_c(i),f1sin_c(i));     % Convert axes coordinates to figure coordinates for 1st axes
    line(f1cos_c(i),f1sin_c(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','b','color','b')

    %  -- 2nd harmonic ---
    plot(x2,y2,'LineWidth',Lw,'Color','r'); hold on;
    line([0 f2cos_c(i)],[0 f2sin_c(i)],'Color','r','LineWidth',Lw,'LineSmoothing','on');
    line(f2cos_c(i),f2sin_c(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','r','color','r')

    [xf2, yf2] = ds2nfu(f2cos_c(i),f2sin_c(i));     % Convert axes coordinates to figure coordinates for 1st axes
    %  -- 3rd harmonic ---
    plot(x3,y3,'LineWidth',Lw,'Color','g'); hold on;
    line([0 f3cos_c(i)],[0 f3sin_c(i)],'Color','g','LineWidth',Lw,'LineSmoothing','on');
    line(f3cos_c(i),f3sin_c(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','r','color','g')

    [xf3, yf3] = ds2nfu(f3cos_c(i),f3sin_c(i));     % Convert axes coordinates to figure coordinates for 1st axes
    
     %  -- 4th harmonic ---
    plot(x4,y4,'LineWidth',Lw,'Color','m'); hold on;
    line([0 f4cos_c(i)],[0 f4sin_c(i)],'Color','m','LineWidth',Lw,'LineSmoothing','on');
    line(f4cos_c(i),f4sin_c(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','r','color','m')

    [xf4, yf4] = ds2nfu(f4cos_c(i),f4sin_c(i));     % Convert axes coordinates to figure coordinates for 1st axes

%     title('Harmonic Circles','FontSize',Fs,'FontName','Century Gothic')
%     set(gca,'FontSize',Fs,'FontName','Century Gothic'); %'Aharoni');
    
%% Converting the harmonics
    sp2=subplot(1,2,2);
    %  -- 1st harmonic ---
    
    plot(x(1:i),f1sin(1:i),'LineWidth',Lw,'Color','b'); hold on; grid on;

    ylim([-2.5 2.5]); xlim([0 19])
    set(sp2,'Position',[0.48    0.178200    0.49    0.680]);

    % Convert axes coordinates to figure coordinates for 1st axes
    [xg1, yg1] = ds2nfu(x(i),f1sin(i));
    annotation('line',[xf1 xg1],[yf1 yg1],'color','b','LineStyle','--','LineWidth',Lw);

    %  -- 2nd harmonic ---
    plot(sp2,x(1:i),f2sin(1:i),'g','LineWidth',Lw,'Color','r'); hold on; grid on;

    [xg2, yg2] = ds2nfu(x(i),f2sin(i)); % Convert axes coordinates to figure coordinates for 1st axes
    annotation('line',[xf2 xg2],[yf2 yg2],'color','r','LineStyle','--','LineWidth',Lw);

    %  -- 3rd harmonic ---
    plot(sp2,x(1:i),f3sin(1:i),'g','LineWidth',Lw,'Color','g'); hold on; grid on;

    [xg3, yg3] = ds2nfu(x(i),f3sin(i)); % Convert axes coordinates to figure coordinates for 1st axes
    annotation('line',[xf3 xg3],[yf3 yg3],'color','g','LineStyle','--','LineWidth',Lw);
    %  -- 4th harmonic ---
    plot(sp2,x(1:i),f4sin(1:i),'g','LineWidth',Lw,'Color','m'); hold on; grid on;

    [xg4, yg4] = ds2nfu(x(i),f4sin(i)); % Convert axes coordinates to figure coordinates for 1st axes
    annotation('line',[xf4 xg4],[yf4 yg4],'color','m','LineStyle','--','LineWidth',Lw);
    
    
    line(x(i),f1sin(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','b','color','b')
    line(x(i),f2sin(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','r','color','r')
    line(x(i),f3sin(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','r','color','g')
    line(x(i),f4sin(i),10,'LineStyle','-','MarkerSize',8,'MarkerFaceColor','r','color','m')

%     title('Leg Stride','FontSize',Fs,'FontName','Century Gothic');
%     legend('');
%     xlabel('x','FontSize',Fs,'FontName','Century Gothic');
% 
%     set(gca,'FontSize',Fs,'FontName','Century Gothic'); %'Aharoni');
    % ---------------------------------------------------------------------

% 
%     if (isVideOn == 1)
%         set(gcf,'PaperPositionMode','auto');
% 
%         FolderName='JPEG';
%         mkdir(FolderName)
%         Name=[FolderName '/Animation_'];
%         itx=i;
% %         disp(['itx= ' num2str(itx)]);
%         if (itx <10)
%             print (f1, '-djpeg100', [Name '000' num2str(itx) '.jpg'],'-r96')
%         elseif (itx>=10 && itx <100)
%             print (f1, '-djpeg100', [Name '00' num2str(itx) '.jpg'],'-r96')
%         elseif (itx>=100 && itx <1000)
%             print (f1, '-djpeg100', [Name '0' num2str(itx) '.jpg'],'-r96')
%         else
%             print (f1, '-djpeg100', [Name '' num2str(itx) '.jpg'],'-r96')
%         end
%     end
end