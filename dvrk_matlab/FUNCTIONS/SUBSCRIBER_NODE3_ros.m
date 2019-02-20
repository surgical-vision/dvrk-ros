%% Publisher for Demonstration
node1 = robotics.ros.Node('/needle_position'); % node for publishing 
pubL = robotics.ros.Publisher(node1,'/position_output/left','geometry_msgs/Point');
pubR = robotics.ros.Publisher(node1,'/position_output/right','geometry_msgs/Point');

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

%%
% Sending the message 
reset(rate)
k=1;
needle=[];
while rate.TotalElapsedTime < 10
   
   msgL.X=339+k;
   msgL.Y=285+k;
   msgR.X=309+k;
   msgR.Y=185+k;   
   
   send(pubL,msgL)
   send(pubR,msgR)
   
   needleL(k,1)=posL(1,1);
   needleL(k,2)=posL(1,2);
   needleR(k,1)=posR(1,1);
   needleR(k,2)=posR(1,2);   
   
   k=k+1;
   waitfor(rate);
end
%% 3D reconstruction of the point 
load('stereoParams_cla.mat'); %loading calibration params
load('Text');  % loading params from exstrinsic calibration trasformation

needle_image_3D=triangulate(needleL,needleR,stereoParams); % 3D recontruction of the point

%% Definition of the point in the ws
needle_image_3D_h=needle_image_3D';
needle_image_3D_h(4,:)=ones();

for i=1:size(needle_image_3D,1)
        needle_ws_h(:,i)=inv(Text)*needle_image_3D_h(:,i);
end

%% From ws to RC of PSM
load('Tws_psm');
T_ws_to_rc=T;
for i=1:size(needle_image_3D,1)
    needle_rc_h(:,i)=T_ws_to_rc*needle_ws_h(:,i);
end

%% Motion of the PSM
needle_rc=needle_rc_h(1:3,:);
r = arm('PSM1');

move_translation(r,needle_rc(:,1)')





