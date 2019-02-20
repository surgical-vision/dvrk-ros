%% 3D WORKSPACE-RC(PSM) CALIBRATION

close all
clear all
clc

%% How to start the gui
% Remember to sourse che workspace 
% 0. source/catkin_ws/devel_debug/setup.bash
% 1. Start the application (in the terminal): 
%   rosrun dvrk_robot dvrk_console_json -j /home/davinci/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ucl-daVinci/console-MTMR-PSM1-Teleop.json
% 2. In a new terminal power on the arm with the command: qladisp n1 n2 and
%   press [p] to power the arm. Check in the dvrk window if the arm is
%   correctly powered.
% 3. Go to matlab ... run the comand step by step
%       More info: https://uk.mathworks.com/help/robotics/robot-operating-system-ros.html

%% Creation of the bridge

% rosinit
r = arm('PSM1');


%% POINT AQUISITION - better formulation 
% 0. Set nupoint according the total number of point you would like to use
%       for the workspace calibration
% 1. Locate tool tip on the first point
% 2. Run the matlab loop
% 3. Move the tool tip and press any key for the new acquisition 
% 4. Move the tool in the new position and repeate.

count=1;
numpoint= 11; % num of points acquired for the acquisition + 1 

while (count<numpoint)
    
     c1=r.get_state_joint_current;
     cjoint(:,:,count)=c1(1,1);   % generation of the matrix of joint value

     c1c=r.get_position_current();
     cc(:,:,count)=c1c;    
    
    while ~waitforbuttonpress 
                    
    end                                
      count=count+1;
      close
end



%% SECTION FOR POINT AQUISITION IN TELEOP (remember to save the point at the end)
for d=1
        % Point 1
        c1=r.get_state_joint_current;
        cjoint(:,:,1)=c1(1,1);   % generation of the matrix of joint value

        c1c=r.get_position_current();
        cc(:,:,1)=c1c;      % generation of the matrix of the cartesian values.

        %%
        % Point 2
        c2=r.get_state_joint_current;
        cjoint(:,:,2)=c2(1,1);  

        c2c=r.get_position_current();
        cc(:,:,2)=c2c; 
        %%
        % Point 3
        c3=r.get_state_joint_current;
        cjoint(:,:,3)=c3(1,1);  

        c3c=r.get_position_current();
        cc(:,:,3)=c3c; 
        %%
        % Point 4
        c4=r.get_state_joint_current;
        cjoint(:,:,4)=c4(1,1);  

        c4c=r.get_position_current();
        cc(:,:,4)=c4c; 
        %%
        % Point 5
        c5=r.get_state_joint_current;
        cjoint(:,:,5)=c5(1,1);  

        c5c=r.get_position_current();
        cc(:,:,5)=c5c; 
        %%
        % Point 6
        c6=r.get_state_joint_current;
        cjoint(:,:,6)=c6(1,1);  

        c6c=r.get_position_current();
        cc(:,:,6)=c6c; 
        %%
        % Point 7
        c7=r.get_state_joint_current;
        cjoint(:,:,7)=c7(1,1);  

        c7c=r.get_position_current();
        cc(:,:,7)=c7c; 
        %%
        % Point 8
        c8=r.get_state_joint_current;
        cjoint(:,:,8)=c8(1,1);  

        c8c=r.get_position_current();
        cc(:,:,8)=c8c; 
        %%
        % Point 9
        c9=r.get_state_joint_current;
        cjoint(:,:,9)=c9(1,1);  

        c9c=r.get_position_current();
        cc(:,:,9)=c9c; 
        %%ws=ws./1000;

        % Point 10
        c10=r.get_state_joint_current;
        cjoint(:,:,10)=c10(1,1);  

        c10c=r.get_position_current();
        cc(:,:,10)=c10c; 
        %%
        % Point 11
        c11=r.get_state_joint_current;ws=ws./1000;

        cjoint(:,:,11)=c11(1,1);  

        c11c=r.get_position_current();
        cc(:,:,11)=c11c; 
        %%
        % Point 12
        c12=r.get_state_joint_current;
        cjoint(:,:,12)=c12(1,1);  

        c12c=r.get_position_current();
        cc(:,:,12)=c12c; 
        %%
        % Point 13
        c13=r.get_state_joint_current;
        cjoint(:,:,13)=c13(1,1);  

        c13c=r.get_position_current();
        cc(:,:,13)=c13c; 
        %%
        % Point 14
        c14=r.get_state_joint_current;
        cjoint(:,:,14)=c14(1,1);  

        c14c=r.get_position_current();ws=ws./1000;
        cc(:,:,14)=c14c; 
        %%
        % Point 15
        c15=r.get_state_joint_current;
        cjoint(:,:,15)=c15(1,1);  

        c15c=r.get_position_current();
        cc(:,:,15)=c15c; 
        %%
        % Point 16
        c16=r.get_state_joint_current;
        cjoint(:,:,16)=c16(1,1);  

        c16c=r.get_position_current();
        cc(:,:,16)=c16c; 
        %%
        % Point 17
        c17=r.get_state_joint_current;
        cjoint(:,:,17)=c17(1,1);  ws=ws./1000;

        c17c=r.get_position_current();
        cc(:,:,17)=c17c; 
        %%
        % Point 18
        c18=r.get_state_joint_current;
        cjoint(:,:,18)=c18(1,1);  

        c18c=r.get_position_current();
        cc(:,:,18)=c18c; 
        %%
        % Point 19
        c19=r.get_state_joint_current;
        cjoint(:,:,19)=c19(1,1);  

        c19c=r.get_position_current();
        cc(:,:,19)=c19c; 
        %%
        % Point 20
        c20=r.get_state_joint_current;ws=ws./1000;
        cjoint(:,:,20)=c20(1,1);  

        c20c=r.get_position_current();
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

%% Generatin the mean among the acquisition 
% Mean From 1 to 2
 np=size(cc1,3);

for i=1:np
 
    cc_x(1,1,i)=cc1(1,4,i);for i=1:np
    tele(:,i)=cc(1:3,4,i);
end
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
    
    cc_y(1,1,i)=cc1(2,4,i);ws=ws./1000;
    cc_y(1,2,i)=cc2(2,4,i);
    cc_y(1,3,i)=cc3(2,4,i);
    cc_mean3(2,1,i)=mean(cc_y(:,:,i));
    
    cc_z(1,1,i)=cc1(3,4,i);cc2=cc;

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

%% Better formulation of the points - for calibration and trasformation in m (everything need to be done in meters)

for i=1:np
    tele1(:,i)=teleop1(1:3,4,i); % create 2D matrix with cartesian coordinate
    tele2(:,i)=teleop2(1:3,4,i); 
    tele3(:,i)=teleop3(1:3,4,i); 
    telm2(:,i)=telem2(1:3,1,i); 
    telm3(:,i)=telem3(1:3,1,i);     
        %telm3(:,i)=telem3(1:3,1,i);     
%         tele_prova(:,i)=cc_prova(1:3,1,i);
end
end
%% Writing the points in a 2d dimension
np=numpoint;

for i=1:np-1
    tele(:,i)=cc(1:3,4,i);
end

%% Creation on workspace-points (everything in m )

squareSize= (8.73); % chessboard dimension in m
square=squareSize/1000;

ws=generateCheckerboardPoints([6 3], squareSize);

ws(:,3)=zeros();

ws=ws';
ws=ws./1000;

% We have to invert x and y cause they are swaped 
a=ws(2,:);
ws(2,:)=ws(1,:);
ws(1,:)=a;

%% Visualization of the two points clouds
W=repmat([350],1,1);
w=W(:);
figure(1)
hold on
grid on
for i=1:np
    scatter3(tele(1,i),tele(2,i),tele(3,i),w,'b*');
    hold on
    scatter3(ws(1,i),ws(2,i),ws(3,i),w,'r*');
%   axis equal
    pause(0.1)
end
hold on 
title('Distribution of the aquired points for WS calibration ')
xlabel('x-axe [m]')
ylabel('y-axe [m]')
zlabel('z-aze [m]')


%% Creation of RF
RF_ws=createRF([1*square;0;0],[4*square;0;0],[0;2*square;0;],[0;0;0]);

figure(1)
hold on
% trplot(RF_ws,'length',20,'color','r');    % mm
 trplot(RF_ws,'length',0.02,'color','r');  % meter

%% Determining the transgormation function  + save
[regParams,Bfit,ErrorStats]=absor(ws,tele); % 3xN
M=regParams.M;

T_ws_to_rc=M;
save('T_ws_to_rc_ND','T_ws_to_rc');

RF_rc=T_ws_to_rc*RF_ws;

%% Plotting RF_PSM
figure(1)
hold on
trplot(RF_rc,'length',0.02,'color','b')
%% Checking if the transformation is correct
% Plotting of the points 
ws_h=ws;
ws_h(4,:)=ones();
Y=repmat([150],1,1);
y=Y(:);
for i=1:np
    point_psm_ck(:,i)=T_ws_to_rc*ws_h(:,i);
    figure(1)
    hold on
    scatter3(point_psm_ck(1,i),point_psm_ck(2,i),point_psm_ck(3,i),y,'b','filled')
    hold on
end
axis equal 
%% Quantification of the accuracy
ppoint=tele(1:3,:);
% tpoint=point_psm_final(1:3,:);
tpoint=point_psm_ck(1:3,:);


for i=1:size(tpoint,2)
    
    acc_ws(:,i)=abs(ppoint(:,i)-tpoint(:,i));
    dist_ws(i)=sqrt(acc_ws(1,i)^2+acc_ws(2,i)^2);
    
end

mean_err_ws=mean(dist_ws);
dev_ws=std(dist_ws);









