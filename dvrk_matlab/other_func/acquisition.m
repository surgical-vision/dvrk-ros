%% ACQUISITION OF THE POINT
% This script generate a matrix with all the points acquired 
close all
clear all
clc

%% How to start the gui
% 1. Start the application (in the terminal): 
%   rosrun dvrk_robot dvrk_console_json -j /home/davinci/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ucl-daVinci/console-MTMR-PSM1-Teleop.json
% 2. In a new terminal power on the arm with the command: qladisp n1 n2 and
%   press [p] to power the arm. Check in the dvrk window if the arm is
%   correctly powered.
% 3. Go to matlab ... run the comand step by step
%       More info: https://uk.mathworks.com/help/robotics/robot-operating-system-ros.html
%% Creation of the bridge 

rosinit
rostopic list
r = arm('PSM1');

% %% TRY THIS NEW SESSION FOR ACQUISITION
% 
% cycle= 14;  % insert number of point for acquisition
% 
% for i=1:cycle
%     
%     cj=r.position_joint_current;
%     cjoint(:,:,i)=cj;   % generation of the matrix of joint value 
% 
%     c=r.position_current(1:3,4);
%     cc(:,:,i)=c;      % generation of the matrix of the cartesian values.
% 
%     while waitfor('a')
% %         % waitforbuttonpress returns 0 with click, 1 with key press
% %         % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
%         disp('Press for acquire a new point: you are at %d -point',i);
%     end
% end
% 



%% SECTION FOR POINT AQUISITION IN TELEOP (remember to save the point at the end)
% Point 1
c1=r.position_joint_current;
cjoint(:,:,1)=c1;   % generation of the matrix of joint value 

c1c=r.position_current(1:3,4);
cc(:,:,1)=c1c;      % generation of the matrix of the cartesian values.


%%
% Point 2

c2=r.position_joint_current;
cjoint(:,:,2)=c2;

c2c=r.position_current(1:3,4);
cc(:,:,2)=c2c;

%%
% Point 3


c3=r.position_joint_current;
cjoint(:,:,3)=c3;

c3c=r.position_current(1:3,4);
cc(:,:,3)=c3c;

%%
% Point 4

c4=r.position_joint_current;
cjoint(:,:,4)=c4;

c4c=r.position_current(1:3,4);
cc(:,:,4)=c4c;


%%
% Point 5

c5=r.position_joint_current;
cjoint(:,:,5)=c5;

c5c=r.position_current(1:3,4);
cc(:,:,5)=c5c;


%%
% Point 6

c6=r.position_joint_current;
cjoint(:,:,6)=c6;

c6c=r.position_current(1:3,4);
cc(:,:,6)=c6c;

%%
% Point 7

c7=r.position_joint_current;
cjoint(:,:,7)=c7;

c7c=r.position_current(1:3,4);
cc(:,:,7)=c7c;


%%
% Point 8

c8=r.position_joint_current;
cjoint(:,:,8)=c8;

c8c=r.position_current(1:3,4);
cc(:,:,8)=c8c;

%%
% Point 9

c9=r.position_joint_current;
cjoint(:,:,9)=c9;

c9c=r.position_current(1:3,4);
cc(:,:,9)=c9c;

%%
% Point 10

c10=r.position_joint_current;
cjoint(:,:,10)=c10;

c10c=r.position_current(1:3,4);
cc(:,:,10)=c10c;

%%
% Point 11
c11=r.position_joint_current;
cjoint(:,:,11)=c11;   % generation of the matrix of joint value 

c11c=r.position_current(1:3,4);
cc(:,:,11)=c11c;      % generation of the matrix of the cartesian values.


%%
% Point 12

c12=r.position_joint_current;
cjoint(:,:,12)=c12;

c12c=r.position_current(1:3,4);
cc(:,:,12)=c12c;

%%
% Point 13

c13=r.position_joint_current;
cjoint(:,:,13)=c13;

c13c=r.position_current(1:3,4);
cc(:,:,13)=c13c;

%%
% Point 14

c14=r.position_joint_current;
cjoint(:,:,14)=c14;

c14c=r.position_current(1:3,4);
cc(:,:,14)=c14c;


%%
% Point 15

c15=r.position_joint_current;
cjoint(:,:,15)=c15;

c15c=r.position_current(1:3,4);
cc(:,:,15)=c15c;


%%
% Point 16

c16=r.position_joint_current;
cjoint(:,:,16)=c16;

c16c=r.position_current(1:3,4);
cc(:,:,16)=c16c;

%%
% Point 17

c17=r.position_joint_current;
cjoint(:,:,17)=c17;

c17c=r.position_current(1:3,4);
cc(:,:,17)=c17c;


%%
% Point 18

c18=r.position_joint_current;
cjoint(:,:,18)=c18;

c18c=r.position_current(1:3,4);
cc(:,:,18)=c18c;

%%
% Point 19


c19=r.position_joint_current;
cjoint(:,:,19)=c19;

c19c=r.position_current(1:3,4);
cc(:,:,19)=c19c;

%%
% Point 20

c20=r.position_joint_current;
cjoint(:,:,20)=c20;

c20c=r.position_current(1:3,4);
cc(:,:,20)=c20c;
%%
% Point 21

c21=r.position_joint_current;
cjoint(:,:,21)=c21;

c21c=r.position_current(1:3,4);
cc(:,:,21)=c21c;
%%
% Point 22

c22=r.position_joint_current;
cjoint(:,:,22)=c22;

c22c=r.position_current(1:3,4);
cc(:,:,22)=c22c;
%% Better matrix formulation (loading of the point saved)

load cc1;
load cjoint1;

%% Better formulation of the points
teleop=cc;
np=size(teleop,3);


for i=1:np
    tele(:,i)=teleop(:,:,i); % create 2D matrix with cartesian coordinate
end


for i=1:np
    joint(:,i)=cjoint(:,:,i); % create 2D matrix with joint coordinate
end

%% IN CASE OF MORE THAN ONE AQUISITION
% Creating an avarage of the points taken by teleop

% uploading fileS
load cc1;
cc1=cc;
load cjoint1;
cjoint1=cjoint;

load cc2;
cc2=cc;
load cjoint2;
cjoint2=cjoint;

load cc3;
cc3=cc;
load cjoint3;
cjoint3=cjoint;

load cc4;
cc4=cc;
load cjoint4;
cjoint4=cjoint;

load cc5;
cc5=cc;
load cjoint5;
cjoint5=cjoint;

%% Creation of the avarage measured
 np=size(cc1,3);

%% 
% Mean From 1 to 2
for i=1:np
 
    cc_x(1,1,i)=cc1(1,1,i);
    cc_x(1,2,i)=cc2(1,1,i);
    cc_mean2(1,1,i)=mean(cc_x(:,:,i));
    
    cc_y(1,1,i)=cc1(2,1,i);
    cc_y(1,2,i)=cc2(2,1,i);
    cc_mean2(2,1,i)=mean(cc_y(:,:,i));
    
    cc_z(1,1,i)=cc1(3,1,i);
    cc_z(1,2,i)=cc2(3,1,i);
    cc_mean2(3,1,i)=mean(cc_z(:,:,i));  
end 
%%
% Mean from 1 to 3
for i=1:np
   
    cc_x(1,1,i)=cc1(1,1,i);
    cc_x(1,2,i)=cc2(1,1,i);
    cc_x(1,3,i)=cc3(1,1,i);
    cc_mean3(1,1,i)=mean(cc_x(:,:,i));
    
    cc_y(1,1,i)=cc1(2,1,i);
    cc_y(1,2,i)=cc2(2,1,i);
    cc_y(1,3,i)=cc3(2,1,i);
    cc_mean3(2,1,i)=mean(cc_y(:,:,i));
    
    cc_z(1,1,i)=cc1(3,1,i);
    cc_z(1,2,i)=cc2(3,1,i);
    cc_z(1,3,i)=cc3(3,1,i);
    cc_mean3(3,1,i)=mean(cc_z(:,:,i));

end
%%
% Mean from 1 to 4
for i=1:np
   
    cc_x(1,1,i)=cc1(1,1,i);
    cc_x(1,2,i)=cc2(1,1,i);
    cc_x(1,3,i)=cc3(1,1,i);
    cc_x(1,4,i)=cc4(1,1,i);
    cc_mean4(1,1,i)=mean(cc_x(:,:,i));
    
    cc_y(1,1,i)=cc1(2,1,i);
    cc_y(1,2,i)=cc2(2,1,i);
    cc_y(1,3,i)=cc3(2,1,i);
    cc_y(1,4,i)=cc4(2,1,i);
    cc_mean4(2,1,i)=mean(cc_y(:,:,i));
    
    cc_z(1,1,i)=cc1(3,1,i);
    cc_z(1,2,i)=cc2(3,1,i);
    cc_z(1,3,i)=cc3(3,1,i);
    cc_z(1,4,i)=cc4(3,1,i);
    cc_mean4(3,1,i)=mean(cc_z(:,:,i));

end

% Global mean 
for i=1:np
   
    cc_x(1,1,i)=cc1(1,1,i);
    cc_x(1,2,i)=cc2(1,1,i);
    cc_x(1,3,i)=cc3(1,1,i);
    cc_x(1,4,i)=cc4(1,1,i);
    cc_x(1,5,i)=cc5(1,1,i);

    cc_mean(1,1,i)=mean(cc_x(:,:,i));
    
    cc_y(1,1,i)=cc1(2,1,i);
    cc_y(1,2,i)=cc2(2,1,i);
    cc_y(1,3,i)=cc3(2,1,i);
    cc_y(1,4,i)=cc4(2,1,i);
    cc_y(1,5,i)=cc5(2,1,i);

    cc_mean(2,1,i)=mean(cc_y(:,:,i));
    
    cc_z(1,1,i)=cc1(3,1,i);
    cc_z(1,2,i)=cc2(3,1,i);
    cc_z(1,3,i)=cc3(3,1,i);
    cc_z(1,4,i)=cc4(3,1,i);
    cc_z(1,5,i)=cc5(3,1,i);
    
    cc_mean(3,1,i)=mean(cc_z(:,:,i));

end

%% Creation of the teleop value mediated over the all aquisition

teleop=cc_mean;
teleop=cc_mean3;
telem2=cc_mean2;
telem3=cc_mean3;
telem4=cc_mean4;

teleop1=cc1; % plot of all the points acquired 
teleop2=cc2;
teleop3=cc3;
teleop4=cc4;
teleop5=cc5;

np=length(teleop);
%%
for i=1:np
      tele(:,i)=teleop(:,:,i); % create 2D matrix with cartesian coordinate
%       tele1(:,i)=teleop1(:,:,i); %sto facendo prova
%       tele2(:,i)=teleop2(:,:,i);
%       tele3(:,i)=teleop3(:,:,i);
%       tele4(:,i)=teleop4(:,:,i);
%       tele5(:,i)=teleop5(:,:,i);
%       
%       telm2(:,i)=telem2(:,:,i);
%       telm3(:,i)=telem3(:,:,i);
%       telm4(:,i)=telem4(:,:,i);
end


% for i=1:np
%     joint1(:,i)=cjoint1(:,:,i); % create 2D matrix with joint coordinate
%     joint2(:,i)=cjoint2(:,:,i);
%     joint3(:,i)=cjoint3(:,:,i);
%     joint4(:,i)=cjoint4(:,:,i);
%     joint5(:,i)=cjoint5(:,:,i);
% end


%% Creation of the second cloud of point according to the  chessboard 

% Insert the chess dimension in meters
square= 0.010;
% Insert the number of point along y axes (referred to the normal cartesian axes)
yc= 4;  % it starts from 0!!
% Insert the number of point along y axes (referred to the normal cartesian axes)
xc= 4;

for i=0:(yc-1)
ws(2,1,i+1)=square*i;    % along y-axe
end

ws(2,1,(yc+1):(yc+xc))=square*(yc-1);

for i=1:xc
ws(1,1,(i+yc))=square*i;    % along x-axe
end

ws(1,1,(yc+xc):(yc+xc+(yc-1)))=square*(yc);

for i=1:(yc-1)
ws(2,1,i+(yc+xc))=(square*(yc-1))-(square*i);  % along y-axe
end


for i=1:(xc-1)
ws(1,1,i+(yc+xc+(yc-1)))=(square*(xc))-(square*i);  % along x-axe
end


ws=ws(:,:,1:np); % check to have the right dimension

for i=1:np
    ws_point(:,i)=ws(:,:,i); % creation of a better vector
end

ws_p=zeros(3,np);
ws_p(1:2,1:np)=ws_point; % final cloud of point

 

%% Visualization of the two points clouds
figure(1)
hold on
grid on
for i=1:np
    scatter3(tele(1,i)',tele(2,i)',tele(3,i)','b*');
    hold on
    scatter3(ws_p(1,i),ws_p(2,i),ws_p(3,i),'r*');
    pause(0.1)
end
hold on 
title('Distribution of the aquired points')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')

% figure(1)
% title('Two clouds of points acquired')
% scatter3(tele(1,1:np)',tele(2,1:np)',tele(3,1:np)','b*');
% hold on
% scatter3(ws_p(1,1:np),ws_p(2,1:np),ws_p(3,1:np),'r*');
% hold on
% scatter3(tele1(1,1:np)',tele1(2,1:np)',tele1(3,1:np)','k');
% hold on
% scatter3(tele2(1,1:np)',tele2(2,1:np)',tele2(3,1:np)','k');
% hold on
% scatter3(tele3(1,1:np)',tele3(2,1:np)',tele3(3,1:np)','k');
% hold on
% scatter3(tele4(1,1:np)',tele4(2,1:np)',tele4(3,1:np)','k');
% hold on
% scatter3(tele5(1,1:np)',tele5(2,1:np)',tele5(3,1:np)','k');
%% Definition of the WS ref sistem (RF_ws frame)

RF_ws=createRF(ws_p(:,np),ws_p(:,(np-2)),ws_p(:,3),ws_p(:,1));



figure(1)
hold on
trplot(RF_ws,'length',0.02,'color','r');

%% Mapping the two clouds of points through Horn's methods.
% creation of the transformation matrix

[T] = quaternion_matching(ws_p,tele);   % transformation from ws->rc

RF_rc=T*RF_ws;

%% Other methods -> definition of the error in T estimation

[regParams,Bfit,ErrorStats]=absor(ws_p,tele);
err_mean=ErrorStats.errmax; 
% err_mean(5)=ErrorStats.errmax;     % if i want to know the error on the T estimation
% err_mean(3)=ErrorStats.errmax;     % with just three acquisition


% [regParams,Bfit,ErrorStats]=absor(ws_p,telm2);
% err_mean(2)=ErrorStats.errmax;
% 
% [regParams,Bfit,ErrorStats]=absor(ws_p,telm3);
% err_mean(3)=ErrorStats.errmax;
% [regParams,Bfit,ErrorStats]=absor(ws_p,telm4);
% err_mean(4)=ErrorStats.errmax;
% 
% 
% % Calculation of the error in all the estimation 
% [regParams,Bfit,ErrorStats]=absor(ws_p,tele1);
% err_1=ErrorStats.errmax;  
% [regParams,Bfit,ErrorStats]=absor(ws_p,tele2);
% err_2=ErrorStats.errmax; 
% [regParams,Bfit,ErrorStats]=absor(ws_p,tele3);
% err_3=ErrorStats.errmax; 
% [regParams,Bfit,ErrorStats]=absor(ws_p,tele4);
% err_4=ErrorStats.errmax; 
% [regParams,Bfit,ErrorStats]=absor(ws_p,tele5);
% err_5=ErrorStats.errmax; 
% 
% err_mean(1)=err_1;
% ripetizioni=[1 2 3 4 5];
% % ripetizioni=[1 2 3];
% 
% 
% figure()
% title('Error in Horns method ')
% plot(ripetizioni,err_mean,'color','k');
% grid on

%% Plotting RF_PSM
figure(1)
hold on
trplot(RF_rc,'length',0.02,'color','b')


%% Plotting the movement of RF
figure(1)
hold on
tranimate(RF_ws,RF_rc,'length',0.02,'color','k');


%% Definition of the movement general point

A=[0.025 0.025 0 1]';
A_psm=T*A;

figure(1)
hold on
scatter3(A(1),A(2),A(3),'r')
hold on
scatter3(A_psm(1),A_psm(2),A_psm(3),'b')


%% Traslation of the PSM1
Ac=A_psm(1:3,1);
move_translation(r,Ac')

%% Generation of authomated traj

% Definition of the points 
point(1:3,1)=[0; square; 0];
point(1:3,2)=[square; (square*2); 0];
point(1:3,3)=[square; square; 0];
point(1:3,4)=[(square*2); (square*2); 0];
point(1:3,5)=[(square*2); square; 0];
point(1:3,6)=[(square*3); (square*2); 0];
point(1:3,7)=[(square*3);square; 0];
point(1:3,8)=[(square*4); (square*2); 0];
point(1:3,9)=[(square*4); 0; 0];
point(1:3,10)=[0; 0; 0];
point(4,:)=ones();

% Plotting of the points 
for i=1:10
    point_psm(:,i)=T*point(:,i);
    figure(1)
    hold on
    scatter3(point(1,i),point(2,i), point(3,i),'r')
    hold on
    scatter3(point_psm(1,i),point_psm(2,i),point_psm(3,i),'b')
    hold on
end


%% Motion along the traj of points
pointc=point_psm(1:3,:);

for i=1:10
    move_translation(r,pointc(:,i)')
    pause(0.5)
end











