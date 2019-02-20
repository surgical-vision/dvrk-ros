%% Publisher for DEMO
node1 = robotics.ros.Node('/needle_position'); % node for publishing 
pubL = robotics.ros.Publisher(node1,'/positionL','geometry_msgs/Point');
pubR = robotics.ros.Publisher(node1,'/positionR','geometry_msgs/Point');

% Images are sent already cutted 
msgL=rosmessage('geometry_msgs/Point');
msgL.X=339;
msgL.Y=285;

msgR=rosmessage('geometry_msgs/Point');
msgR.X=309;
msgR.Y=185;

desiredRate = 10;
rate = robotics.Rate(desiredRate);
rate.OverrunAction = 'drop';


%% Definition of subscriber node

% rosinit       % connection with ROS network 

node3 = robotics.ros.Node('/needle_analysis');   % node definition 
% sub = robotics.ros.Subscriber(node2,'/camera','sensor_msgs/Image');     % definition of subscrber IMAGE
sub1 = robotics.ros.Subscriber(node3,'/position_output/left','geometry_msgs/Point',@readCallbackL); % definition of subscriber POINT (rosmsg list u can find even message with orientation)
sub2 = robotics.ros.Subscriber(node3,'/position_output/right','geometry_msgs/Point',@readCallbackR);
global posL
global posR

%% FAKE publisher for testing 
% % Sending the message 
% reset(rate)
% k=1;
% needle=[];
% while rate.TotalElapsedTime < 10
%    
%    msgL.X=339+k;
%    msgL.Y=285+k;
%    msgR.X=309+k;
%    msgR.Y=185+k;   
%    
%    send(pubL,msgL)
%    send(pubR,msgR)
%    
%    needleL(k,1)=posL(1,1);
%    needleL(k,2)=posL(1,2);
%    needleR(k,1)=posR(1,1);
%    needleR(k,2)=posR(1,2);   
%    
%    k=k+1;
%    waitfor(rate);
% end
%% 3D reconstruction of the point 
load('stereoParams_cla_new.mat'); %loading calibration params
load('Text_horn_mm.mat');  % loading params from exstrinsic calibration trasformation
load('T_ws_to_rc');
% T_ws_to_rc=T;

Text=Text_horn_mm;
%% Clear variables
clearvars needleL 
clearvars needleR
clearvars needleL_mean 
clearvars needleR_mean
clearvars needle_image_3D
clearvars needle_ws_h
% clearvars needle_rc_h
clearvars needle_rc
% clearvars needle_ws_m_h
clearvars needle1 
clearvars needle2
clearvars cc
clearvars nee_L
clearvars nee_R
clearvars n_L
clearvars n_R
clearvars n_image_3D
clearvars n_image_3D_h
clearvars n_ws_h
clearvars n_ws_m_h
clearvars n_rc_h
clearvars n_rc
clearvars n_pos
%% Making avarage of needle position 
for d=1
% Acquisition of 10 different positions
for k=1:10
    needleL(k,1)=posL(1,1);
    needleL(k,2)=posL(1,2);    
    needleR(k,1)=posR(1,1);
    needleR(k,2)=posR(1,2);
    
end

% Making the avarage 
needleL_mean(1,1)=mean(needleL(:,1),1);
needleL_mean(1,2)=mean(needleL(:,2),1);
needleR_mean(1,1)=mean(needleR(:,1),1);
needleR_mean(1,2)=mean(needleR(:,2),1);

% POINT RECONSTRUCTED IN MM
needle_image_3D=triangulate(needleL_mean,needleR_mean,stereoParams); % 3D recontruction of the point


% Definition of the point in the ws
needle_image_3D_h=needle_image_3D';
needle_image_3D_h(4,:)=ones();

% For just one point at the time 
% IMPORTANT : THOSE MEASUREMENT ARE IN MM
needle_ws_h(:,1)=inv(Text_horn_mm)*needle_image_3D_h(:,1);
% needle_ws_h(:,1)=Text*needle_image_3D_h(:,1);

% In case of more than one point
% for i=1:size(needle_image_3D,1)
%         needle_ws_h(:,i)=inv(Text)*needle_image_3D_h(:,i);
% end
% Conversion from mm to m
needle_ws_m_h(1:3)=needle_ws_h(1:3)./1000;
needle_ws_m_h(4)=ones();
% needle_ws_m_h=needle_ws_m_h';

%
needle_rc_h(:,1)=T_ws_to_rc*needle_ws_m_h(:,1);

% for i=1:size(needle_image_3D,1)
%     needle_rc_h(:,i)=T_ws_to_rc*needle_ws_h(:,i);
% end

%  coordinate formulation
needle_rc=needle_rc_h(1:3,:);
%r = arm('PSM1');


% DEFINITION OF THE 2-STAGE MOTION
needle1(1:2,1)=needle_rc(1:2,1);
needle1(3,1)=needle_home_rc(3,1);

needle2(1:2,1)=needle1(1:2,1);
needle2(3,1)=needle_rc(3,1);

% MOTION
% STEP 1
move_translation(r,needle1(:,1)')
%
% STEP 2
move_translation(r,needle2(:,1)')




%% COMPLETE MOTION
move_translation(r,needle1(:,1)') % position same plane
pause(0.1)
move_translation(r,needle2(:,1)')   % going into deep
pause(0.1)
r.dmove_joint_one(-0.90, int8(7))   % grasping
pause(0.8)
r.dmove_joint_one(0.90, int8(7))    % release
pause(0.1)
move_translation(r,needle1(:,1)') % going back
pause(0.1)
move_translation(r,home0_psm(:,1)') % going home

%% back home
move_translation(r,needle1(:,1)')
pause(0.05)
move_translation(r,home0_psm(:,1)')




%% Look into
for d=1
%% Opening the gripper 
r.dmove_joint_one(1.1, int8(7))

%% MOTION - Generation of the motion 
move_translation(r,needle_rc(:,1)')

%% Closing the gripper 
r.dmove_joint_one(-1.1, int8(7))

%% MOTION COMPLETE
move_translation(r,needle_rc(:,1)')
pause(0.5)
r.dmove_joint_one(-0.90, int8(7))
pause(1)
r.dmove_joint_one(0.90, int8(7))
pause(0.3)
move_translation(r,home0_psm(:,1)')



%% DEFINITION OF HOME POSITION
home0(1:3,1,1)=[3*square;3*square;6*square];
home0(4,:)=ones();
home0_psm(:,1)=T_ws_to_rc*home0(:,1);
home0_psm=home0_psm(1:3,:);

home1(1:3,1,1)=[8*square;8*square;8*square];
home1(4,:)=ones();
home1_psm(:,1)=T_ws_to_rc*home1(:,1);
home1_psm=home1_psm(1:3,:);
%% Move to home position
move_translation(r,home0_psm(:,1)')
%% needle_home_rc=[-0.0168 -0.0241 -0.1084]';

%%
move_translation(r,home1_psm(:,1)')

%% movimento composito 
needle1(1:2,1)=needle_rc(1:2,1);
needle1(3,1)=needle_home_rc(3,1);

needle2(1:2,1)=needle1(1:2,1);
needle2(3,1)=needle_rc(3,1);

%%
move_translation(r,needle1(:,1)')
%%
move_translation(r,needle2(:,1)')

%% mettendo tutto insieme
move_translation(r,needle1(:,1)') % position same plane
pause(0.1)
move_translation(r,needle2(:,1)')   % going into deep
pause(0.1)
r.dmove_joint_one(-0.90, int8(7))   % grasping
pause(0.8)
r.dmove_joint_one(0.90, int8(7))    % release
pause(0.1)
move_translation(r,needle1(:,1)') % going back
pause(0.1)
move_translation(r,home0_psm(:,1)') % going home

end 
end
%%
for d=1
rot = stereoParams.RotationOfCamera2;
tra = stereoParams.TranslationOfCamera2;
camMatrix1 = cameraMatrix(stereoParams.CameraParameters1, eye(3), [0 0 0]);
camMatrix2 = cameraMatrix(stereoParams.CameraParameters2, rot, tra);
end

for d=1
%% General motion
%% Follow the needle 

% initialization part 
needleL(1,1)=posL(1,1);
needleL(1,2)=posL(1,2);    
needleR(1,1)=posR(1,1);
needleR(1,2)=posR(1,2);
needle_image_3D=triangulate(needleL,needleR,stereoParams); % 3D recontruction of the point
needle_image_3D_h=needle_image_3D';
needle_image_3D_h(4,:)=ones();
needle_ws_h(:,1)=inv(Text_horn_mm)*needle_image_3D_h(:,1);
needle_ws_m_h(1:3)=needle_ws_h(1:3)./1000;
needle_ws_m_h(4)=ones();
needle_rc_h(:,1)=T_ws_to_rc*needle_ws_m_h(:,1);
needle_rc=needle_rc_h(1:3,:);

% Reaching the initial position 
needle1(1:2,1)=needle_rc(1:2,1);
needle1(3,1)=needle_home_rc(3,1);

move_translation(r,needle1(:,1)')
pause(3)

% Following the needle
n_pos(:,:,1)=needle1;
n_pos(3,:,1:500)=needle1(3,1);
diff(1:3,1,1)=n_pos(:,:,1);
n_deep(1:2,1)=needle1(1:2,1);
k=1;
%



%%

while(1)
   
    disp('START THE MOTION OF THE NEEDLE')
%     n_L(:,:,k)=posL;
%     n_R(:,:,k)=posR;
    pause(1)
    for q=1:10
        nee_L(q,1,k)=posL(1,1);
        nee_L(q,2,k)=posL(1,2);    
        nee_R(q,1,k)=posR(1,1);
        nee_R(q,2,k)=posR(1,2);
    end
    % Making the avarage
    n_L(1,1,k)=mean(nee_L(:,1,k),1);
    n_L(1,2,k)=mean(nee_L(:,2,k),1);
    n_R(1,1,k)=mean(nee_R(:,1,k),1);
    n_R(1,2,k)=mean(nee_R(:,2,k),1);
    
%     n_L_u(:,:,k)=undistortPoints(n_L(:,:,k),stereoParams.CameraParameters1);
%     n_R_u(:,:,k)=undistortPoints(n_R(:,:,k),stereoParams.CameraParameters2);
    n_L_u(:,:,k)=n_L(:,:,k);
    n_R_u(:,:,k)=n_R(:,:,k);
%     [n_image_3D(:,:,k),reprojerror]=triangulate(n_L_u(:,:,k),n_R_u(:,:,k),camMatrix1, camMatrix2);
    n_image_3D(:,:,k)=triangulate(n_L_u(:,:,k),n_R_u(:,:,k),stereoParams); % 3D recontruction of the point
    n_image_3D_h(1:3,:,k)=n_image_3D(:,:,k)';
    n_image_3D_h(4,:,k)=ones();
    n_ws_h(:,1,k)=inv(Text_horn_mm)*n_image_3D_h(:,1,k);
    n_ws_m_h(1:3,1,k)=n_ws_h(1:3,1,k)./1000;
    n_ws_m_h(4,1,k)=ones();
    n_rc_h(:,1,k)=T_ws_to_rc*n_ws_m_h(:,1,k);
    n_rc(:,:,k)=n_rc_h(1:3,:,k);
    n_pos(1:2,1,k+1)=n_rc(1:2,1,k); % definition of the position
    diff(1:2,1,k+1)= n_pos(1:2,1,k+1)-n_pos(1:2,1,k); % calculating difference in position
    move_translation(r,n_pos(:,1,k+1)') % motion of the robot
    
    if(abs((diff(1,1,k+1)+ diff(1,1,k)))< 0.001 && abs((diff(2,1,k+1)+diff(2,1,k)))< 0.001 ) 
%         n_deep(3,1)=n_rc(3,1,k);
        move_translation(r,n_rc(:,1,k)')
        disp('end of experiment')
        break;
    end
    
    k=k+1;

end

%% back home
move_translation(r,needle1(:,1)')
pause(0.05)
move_translation(r,home0_psm(:,1)')
end