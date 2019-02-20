%% CAMERA POSE ESTIMATION

% It can be applied to both Left and Right endoscopic camera (if you would
% like to change to right, please uncomment the code!).

%% In case of acquisition of the frame from ROS
% rosinit 
% image acquisition 
image_sub_raw=rossubscriber('/image_output/left/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'LEFT-P.png');

image_sub_raw=rossubscriber('/image_output/right/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'RIGHT-P.png');

%%
% Generation of the undistort images 
load('stereoParamsD.mat');  %C
camera1= stereoParamsD.CameraParameters1;   %C
camera2= stereoParamsD.CameraParameters2;   %C
     
L_cam_grey= imread('LEFT-P_s.png');
L_cam = undistortImage(L_cam_grey, camera1); 
R_cam_grey= imread('RIGHT-P_s.png');
R_cam = undistortImage(R_cam_grey, camera2);

% detect 2D checkerboard points
[ L_imagePoints , L_boardSize ] = detectCheckerboardPoints(L_cam) ;
[ R_imagePoints , R_boardSize ] = detectCheckerboardPoints(R_cam) ;

figure(1); 
title('Checkboards detection for left and right camera')
subplot(1,2,1)
imshow(L_cam)
hold on
plot(L_imagePoints(:,1),L_imagePoints(:,2),'r*');
subplot(1,2,2)
imshow(R_cam)
hold on
plot(R_imagePoints(:,1),R_imagePoints(:,2),'b*');
pause(3)
close

% Reconstruction of the point in 3D : point in mm express respect camera1
% in mm
worldPoints_check=triangulate(L_imagePoints,R_imagePoints,stereoParamsD);   %C
worldPoints_check=worldPoints_check./1000; % transformation in m 

% Generate checkboard in /ws
squareSize= (8.73); % chessboard dimension in mm
square=squareSize/1000; % definition in m
ws_check=generateCheckerboardPoints(L_boardSize, square);
ws_check(:,3)=zeros();

figure(2)
title('Plotting the point reconstruct in the LEFT camera sistem and real points [m]')
for i=1:size(ws_check,1)
    scatter3(worldPoints_check(i,1),worldPoints_check(i,2),worldPoints_check(i,3),'k*');
    grid on
    hold on
    scatter3(ws_check(i,1),ws_check(i,2),ws_check(i,3),'k','filled');
    pause(0.01)
end

% Determining the camera pose transformation
[regParams,Bfit,ErrorStats]=absor(ws_check',worldPoints_check'); 
T_err_mm=regParams.M;
worldPoints_check_h=worldPoints_check';
worldPoints_check_h(4,:)=ones();
world_err_check=inv(T_err_mm)*worldPoints_check_h;
save('T_cam_pose_mm','T_err_mm');

figure(5)
title('Plotting real point, reconstructed ones, back projected ones - [m]')
hold on
grid on
for i=1:size(worldPoints_check,1)
    scatter3(worldPoints_check(i,1),worldPoints_check(i,2),worldPoints_check(i,3),'ko');
    hold on
    scatter3(world_err_check(1,i),world_err_check(2,i),world_err_check(3,i),'b*');
    hold on
    scatter3(ws_check(i,1),ws_check(i,2),ws_check(i,3),'k','filled');
    pause(0.01)
end
legend('triangulate points','T-cam-pos applied','real point /ws')
hold off


