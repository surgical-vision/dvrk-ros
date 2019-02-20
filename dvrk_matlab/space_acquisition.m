%% 3D CALIBRATION

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

%% SECTION FOR POINT AQUISITION IN TELEOP (remember to save the point at the end)
% Point 1
c1=r.position_joint_current;
cjoint(:,:,1)=c1;   % generation of the matrix of joint value 

c1c=r.position_current(:,:);
cc(:,:,1)=c1c;      % generation of the matrix of the cartesian values.

%%
% Point 2
c2=r.position_joint_current;
cjoint(:,:,2)=c2;  

c2c=r.position_current(:,:);
cc(:,:,2)=c2c; 
%%
% Point 3
c3=r.position_joint_current;
cjoint(:,:,3)=c3;  

c3c=r.position_current(:,:);
cc(:,:,3)=c3c; 
%%
% Point 4
c4=r.position_joint_current;
cjoint(:,:,4)=c4;  

c4c=r.position_current(:,:);
cc(:,:,4)=c4c; 
%%
% Point 5
c5=r.position_joint_current;
cjoint(:,:,5)=c5;  

c5c=r.position_current(:,:);
cc(:,:,5)=c5c; 
%%
% Point 6
c6=r.position_joint_current;
cjoint(:,:,6)=c6;  

c6c=r.position_current(:,:);
cc(:,:,6)=c6c; 
%%
% Point 7
c7=r.position_joint_current;
cjoint(:,:,7)=c7;  

c7c=r.position_current(:,:);
cc(:,:,7)=c7c; 
%%
% Point 8
c8=r.position_joint_current;
cjoint(:,:,8)=c8;  

c8c=r.position_current(:,:);
cc(:,:,8)=c8c; 
%%
% Point 9
c9=r.position_joint_current;
cjoint(:,:,9)=c9;  

c9c=r.position_current(:,:);
cc(:,:,9)=c9c; 
%%
% Point 10
c10=r.position_joint_current;
cjoint(:,:,10)=c10;  

c10c=r.position_current(:,:);
cc(:,:,10)=c10c; 
%%
% Point 11
c11=r.position_joint_current;
cjoint(:,:,11)=c11;  

c11c=r.position_current(:,:);
cc(:,:,11)=c11c; 
%%
% Point 12
c12=r.position_joint_current;
cjoint(:,:,12)=c12;  

c12c=r.position_current(:,:);
cc(:,:,12)=c12c; 
%%
% Point 13
c13=r.position_joint_current;
cjoint(:,:,13)=c13;  

c13c=r.position_current(:,:);
cc(:,:,13)=c13c; 
%%
% Point 14
c14=r.position_joint_current;
cjoint(:,:,14)=c14;  

c14c=r.position_current(:,:);
cc(:,:,14)=c14c; 
%%
% Point 15
c15=r.position_joint_current;
cjoint(:,:,15)=c15;  

c15c=r.position_current(:,:);
cc(:,:,15)=c15c; 
%%
% Point 16
c16=r.position_joint_current;
cjoint(:,:,16)=c16;  

c16c=r.position_current(:,:);
cc(:,:,16)=c16c; 
%%
% Point 17
c17=r.position_joint_current;
cjoint(:,:,17)=c17;  

c17c=r.position_current(:,:);
cc(:,:,17)=c17c; 
%%
% Point 18
c18=r.position_joint_current;
cjoint(:,:,18)=c18;  

c18c=r.position_current(:,:);
cc(:,:,18)=c18c; 
%%
% Point 19
c19=r.position_joint_current;
cjoint(:,:,19)=c19;  

c19c=r.position_current(:,:);
cc(:,:,19)=c19c; 
%%
% Point 20
c20=r.position_joint_current;
cjoint(:,:,20)=c20;  

c20c=r.position_current(:,:);
cc(:,:,20)=c20c; 

%% Better matrix formulation (loading of the point saved)
load cc1;
load cjoint1;
cc1=cc;
cjoint1=cjoint;
%%
load cc2;
load cjoint2;
cc2=cc;
cjoint2=cjoint;
%%
load cc3;
load cjoint3;
cc3=cc;
cjoint3=cjoint;

%%
% Mean From 1 to 2
 np=size(cc1,3);

for i=1:np
 
    cc_x(1,1,i)=cc1(1,4,i);
    cc_x(1,2,i)=cc2(1,4,i);
    cc_mean2(1,1,i)=mean(cc_x(:,:,i));
    
    cc_y(1,1,i)=cc1(2,4,i);
    cc_y(1,2,i)=cc2(2,4,i);
    cc_mean2(2,1,i)=mean(cc_y(:,:,i));
    
    cc_z(1,1,i)=cc1(3,4,i);
    cc_z(1,2,i)=cc2(3,4,i);
    cc_mean2(3,1,i)=mean(cc_z(:,:,i));  
end 
%%
% Mean from 1 to 3
for i=1:np
   
    cc_x(1,1,i)=cc1(1,4,i);
    cc_x(1,2,i)=cc2(1,4,i);
    cc_x(1,3,i)=cc3(1,4,i);
    cc_mean3(1,1,i)=mean(cc_x(:,:,i));
    
    cc_y(1,1,i)=cc1(2,4,i);
    cc_y(1,2,i)=cc2(2,4,i);
    cc_y(1,3,i)=cc3(2,4,i);
    cc_mean3(2,1,i)=mean(cc_y(:,:,i));
    
    cc_z(1,1,i)=cc1(3,4,i);
    cc_z(1,2,i)=cc2(3,4,i);
    cc_z(1,3,i)=cc3(3,4,i);
    cc_mean3(3,1,i)=mean(cc_z(:,:,i));

end

%%
telem2=cc_mean2;
telem3=cc_mean3;

teleop1=cc1; % plot of all the points acquired 
teleop2=cc2;
teleop3=cc3;

%% Better formulation of the points - for calibration


for i=1:np
%     tele1(:,i)=teleop1(1:3,4,i); % create 2D matrix with cartesian coordinate
%     tele2(:,i)=teleop2(1:3,4,i); 
%     tele3(:,i)=teleop3(1:3,4,i); 
%     telem2(:,i)=telem2(1:3,1,i); 
    telm3(:,i)=telem3(1:3,1,i);     
end


% for i=1:np np=size(cc1,3);

%     joint(:,i)=cjoint(:,:,i); % create 2D matrix with joint coordinate
% end

%% Creation on point-cloud
square= 0.010;

ws(1:3,1)=[5*square;0*square;1*square];
ws(1:3,2)=[5*square;5*square;1*square];
ws(1:3,3)=[0*square;5*square;1*square];
ws(1:3,4)=[4*square;0*square;2*square];
ws(1:3,5)=[4*square;4*square;2*square];
ws(1:3,6)=[0*square;4*square;2*square];
ws(1:3,7)=[3*square;0*square;3*square];
ws(1:3,8)=[3*square;3*square;3*square];
ws(1:3,9)=[0*square;3*square;3*square];

% ws(1:3,1)=[0*square;3*square;1*square];
% ws(1:3,2)=[5*square;3*square;1*square];
% ws(1:3,3)=[5*square;8*square;1*square];
% ws(1:3,4)=[0*square;4*square;2*square];
% ws(1:3,5)=[4*square;4*square;2*square];
% ws(1:3,6)=[4*square;8*square;2*square];
% ws(1:3,7)=[0*square;5*square;3*square];
% ws(1:3,8)=[3*square;5*square;3*square];
% ws(1:3,9)=[3*square;8*square;3*square];
% ws(1:3,10)=[0*square;6*square;4*square];
% ws(1:3,11)=[2*square;6*square;4*square];
% ws(1:3,12)=[2*square;8*square;4*square];




%% Visualization of the two points clouds
W=repmat([350],1,1);
w=W(:);
figure(1)
hold on
grid on
for i=1:np
    scatter3(telm3(1,i),telm3(2,i),telm3(3,i),w,'b*');
    hold on
    scatter3(ws(1,i),ws(2,i),ws(3,i),w,'r*');
    axis equal
    pause(0.1)
end
hold on 
title('Distribution of the aquired points')
xlabel('x-axe [m]')
ylabel('y-axe [m]')
zlabel('z-aze [m]')

% figure(1)
% hold on
% scatter3(tele1(1,1:np)',tele1(2,1:np)',tele1(3,1:np)','k','filled');
% hold on
% scatter3(tele2(1,1:np)',tele2(2,1:np)',tele2(3,1:np)','k','filled');
% hold on
% scatter3(tele3(1,1:np)',tele3(2,1:np)',tele3(3,1:np)','k','filled');



%% Create RF

RF_ws=createRF([1*square;0;0],[4*square;0;0],[0;2*square;0],[0;0;0]);

figure(1)
hold on
trplot(RF_ws,'length',0.02,'color','r');

%% Mapping the two clouds of points through Horn's methods.
% creation of the transformation matrix

[T] = quaternion_matching(ws,telem3);   % transformation from ws->rc

RF_rc=T*RF_ws;


%% Plotting RF_PSM
figure(1)
hold on
trplot(RF_rc,'length',0.02,'color','b')



%% Checking if the transformation is correct
% Plotting of the points 
ws_h=ws;
ws_h(4,:)=ones();

 
R=repmat([150],1,1);
r=R(:);
for i=1:np
    point_psm_ck(:,i)=T*ws_h(:,i);
    figure(1)
    hold on
    scatter3(point_psm_ck(1,i),point_psm_ck(2,i),point_psm_ck(3,i),r,'b','filled')
    hold on
end



%% PROVA NUOVA
point(1:3,1)=[0*square; 8*square; 5*square];
point(1:3,2)=[1*square; 7*square; 5*square];
point(1:3,3)=[4*square; 4*square; 6*square];
point(1:3,4)=[2*square; 2*square; 2*square];
point(1:3,5)=[5*square; 5*square; 3*square];
point(1:3,6)=[6*square; 2*square; 0*square];
% point(1:3,7)=[3*square; 3*square; 2*square];
% point(1:3,8)=[4*square; 4*square; 2*square];
% point(1:3,9)=[4*square; 4*square; 1*square];

% point(1:3,3)=[1*square; 1*square; 4*square];
% point(1:3,4)=[2*square; 2*square; 4*square];
% point(1:3,5)=[2*square; 2*square; 3*square];
% point(1:3,6)=[3*square; 3*square; 3*square];
% point(1:3,7)=[3*square; 3*square; 2*square];
% point(1:3,8)=[4*square; 4*square; 2*square];
% point(1:3,9)=[4*square; 4*square; 1*square];
% point(1:3,10)=[5*square; 5*square; 1*square];
% point(1:3,11)=[6*square; 6*square; 0*square];
point(4,:)=ones();
% Plotting of the points 
for i=1:size(point,2)
    point_psm(:,i)=T*point(:,i);
    figure(1)
    hold on
    scatter3(point(1,i),point(2,i), point(3,i),'r','filled')
    hold on
    scatter3(point_psm(1,i),point_psm(2,i),point_psm(3,i),'b','filled')
    hold on
end

home(1:3,1,1)=[8*square;8*square;8*square];
home(4,:)=ones();
home_psm(:,1)=T*home(:,1);

%% Motion along the traj of points
pointc=point_psm(1:3,:);
home_psm=home_psm(1:3,:);

%%
move_translation(r,home_psm(:,1)')
%% Pose validation
for i=1:size(pointc,2)
    move_translation(r,pointc(:,i)')
    pause(0.5)
end

%% Rotation validation 
[Q,R]=qr(rand(3));
frame1(1:3,1:3)=Q;
frame1(1:3,4)=[5*square;5*square;5*square];
frame1(4,1:4)=[0 0 0 1];

frame1_psm=T*frame1;

%%
[Q,R]=qr(rand(3));
frame2(1:3,1:3)=Q;
frame2(1:3,4)=[4*square;4*square;4*square];
frame2(4,1:4)=[0 0 0 1];

frame2_psm=T*frame2;

[Q,R]=qr(rand(3));
frame3(1:3,1:3)=Q;
frame3(1:3,4)=[5*square;5*square;6*square];
frame3(4,1:4)=[0 0 0 1];

frame3_psm=T*frame3;

%% Opening the gripper
move_joint_one(r, 0.8 ,int8(7))
%% Rotation motion
move(r,frame1_psm)
%%
move(r,frame2_psm)
%%
move(r,frame3_psm)


