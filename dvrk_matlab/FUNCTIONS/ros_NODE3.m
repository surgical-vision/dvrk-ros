%% Creation of a publisher
rosinit     %initialization ros
image_pub=rospublisher('/camera','sensor_msgs/Image'); % generation of a topi /camera
needle = rosmessage('sensor_msgs/Image');
% Creation of ROS message 
needle.Encoding = 'rgb8';
image=imread(['Image' int2str(1), '.jpg']);
writeImage(needle,image);
send(image_pub,needle)

%%
image_sub=rossubscriber('/camera');
msgImg = receive(image_sub);
needle_read = readImage(msgImg);






%% Creation of ROS node able to acquire information 
needlepose=rossubscriber('/topicname');
% if topic already exist it should alredy nome message type

needle_image=receive(needlepose,10); %2D coordinate of the point in the image plane 

load('stereoParams_cla.mat'); %loading calibration params
needle_image_3D=triangulate(needle_image()',needle_image()',stereoParams); % 3D recontruction of the point
% loading params from exstrinsic calibration trasformation
load('Text');

%% Definition of the point in the ws
needle_image_3D_h=needle_image_3D;
needle_image_3D_h(4,:)=ones();

for i=1:size(needle_image_3D,2)
        needle_ws_h(:,i)=inv(Text)*needle_image_3D_h(:,i);
end

% From ws to RC of PSM
for i=1:size(needle_image_3D,2)
    needle_rc_h(:,i)=T_ws_to_rc*needle_ws_h(:,i);
end

% Motion of the PSM
needle_rc=needle_rc_h(1:3,:);

move_translation(r,needle_rc(:,i)')

