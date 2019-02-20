%% This script allows to test the transformation coming from the workspace calibration and the camera pose estimation. 
% The point is selected in the image and reconstruct and transpose in the
% /rc and used to generate the motion of the robot
%% Try all the transformation together
% point acquisition
load('stereoParamsD.mat'); %loading calibration params
load('T_err_mm.mat');
camera1= stereoParamsD.CameraParameters1;
camera2= stereoParamsD.CameraParameters2;
%% Acquire, save and undistort the image
image_sub_raw=rossubscriber('/image_output/left/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'LEFT-PAF1.png');

image_sub_raw=rossubscriber('/image_output/right/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'RIGHT-PAF1.png');
%
IL= imread('LEFT-PAF1.png');
IR= imread('RIGHT-PAF1.png');
ILu = undistortImage(IL,camera1);
IRu = undistortImage(IR,camera2);

%Selecting the points - manually 

myfig=figure(30);
hold on
% LEFT CAMERA 

% Left camera
for j=1:1 %size(frame_st,2)


    imshow(ILu);
    cursorobj = datacursormode(myfig);

    cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

    cursorobj.Enable = 'off';

    mypoints(j) = getCursorInfo(cursorobj);

    cc_try_L(:,j)=mypoints(j).Position;
end


% RIGHT CAMERA
 % right camera
for j=1:1 %size(frame_st,2)


    imshow(IRu);
    cursorobj = datacursormode(myfig);

    cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

    cursorobj.Enable = 'off';

    mypoints(j) = getCursorInfo(cursorobj);

    cc_try_R(:,j)=mypoints(j).Position;
end

close

tryPoints=triangulate(cc_try_L',cc_try_R',stereoParamsD);
tryPoints=tryPoints./1000 

%Definition of the point in the ws

tryPoints_h=tryPoints';
tryPoints_h(4,:)=ones();
%
for i=1:size(tryPoints_h,2)
        try_ws_h(:,i)=inv(T_err)*tryPoints_h(:,i);
end

try_ws_h(3,1)=try_ws_h(3,1)%+0.010;

%
% From ws to RC of PSM
for i=1:size(tryPoints_h,2)
    try_rc_h(:,i)=T_ws_to_rc*try_ws_h(:,i);
end

%
% Motion of the PSM
try_rc=try_rc_h(1:3,:)

% try_rc(3,1)=-0.1451
% figure()
% grid on
% scatter()


%% Grasping the target
pause(0.3)
% move_translation(r,home_psm(1:3,1)')
%
open_jaw(R)
move_translation(r,try_rc(:,1)')% getting the rail

%
pause(1)
close_jaw(R)
move_translation(r,home_psm(1:3,1)') % home position


%

move_joint(r,c1_kid') % locatin in the kid
pause(2)
open_jaw(R)

%
move_joint(r,[0.7268   -0.0108    0.1060    3.2800   -0.0271    0.0406])
close_jaw(R)


%%
move_translation(r,try_rc(:,1)')
pause(0.4)
open_jaw(R)
move_translation(r,home_psm(1:3,1)')
close_jaw(R)
%% Location on the kidnet 
move_joint(r,c1_kid')

%% Release back the tarfet in the starting position
pause(1)
move_translation(r,try_rc(:,1)')
pause(0.4)
open_jaw(R)
move_translation(r,home_psm(1:3,1)')
close_jaw(R)

%% Joint motion 
one= 0.0;
two= 0.0 ;
three= 0.0;
four= 0.0;  % skew rotation
five= 0.0;  % plam orientation
six= 0.4;  % fingers orientation
dmove_joint(r,[one two three four five six])

%% Joint configuration in home posiziont
open_jaw(R)
move_joint(r,[0.7268   -0.0108    0.1060    3.2800   -0.0271    0.0406])
close_jaw(R)

%% Point of kidney
% 
c1_kid=r.get_state_joint_current;
c1c_kid=r.get_position_current();
cart_kid=c1c_kid(1:3,4);
% move_joint(r,c1_kid')

%%
c1_rail=r.get_state_joint_current;
c1c_rail=r.get_position_current();
cart_rail=c1c_rail(1:3,4);
% move_joint(r,c1_kid')

%%
move_joint(r,c1_rail')
close_jaw(R)

%%
c1c_h=r.get_position_current();
cart_h=c1c_h(1:3,4);
