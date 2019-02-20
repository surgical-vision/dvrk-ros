%% Verify all the transformations 

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
% generation of the undistort images 
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


% handles.H=
figure(1); % to close the image pressin a botton
subplot(1,2,1)
title('Checkboards detection for left and right camera')
imshow(L_cam)
hold on
plot(L_imagePoints(:,1),L_imagePoints(:,2),'r*');
subplot(1,2,2)
imshow(R_cam)
hold on
plot(R_imagePoints(:,1),R_imagePoints(:,2),'b*');
% waitforbuttonpress;
% close(handles.H)

%
% Reconstruction of the point in 3D : point in mm express respect camera1
worldPoints_check=triangulate(L_imagePoints,R_imagePoints,stereoParamsD);   %C

% handles.H=figure(2);
figure(2)
for i=1:size(worldPoints_check,1)
%     plot(L_imagePoints(i,1),L_imagePoints(i,2),'r*');
    hold on
    grid on
%     plot(R_imagePoints(i,1),R_imagePoints(i,2),'b*');
%     hold on
    scatter3(worldPoints_check(i,1),worldPoints_check(i,2),worldPoints_check(i,3),'k*');
    %pause(0.1)
end
axis equal
% waitforbuttonpress;
% close(handles.H)


%
% Trasform the point from camera system to /ws
load('T_cam_poseD.mat');    %C
worldPoints_check_h=worldPoints_check';
worldPoints_check_h(4,:)=ones();

load('T_cam_poseD.mat');    %C
ws_check_cam= inv(T_cam_pose)*worldPoints_check_h;


% Generate checkboard in /ws
squareSize= (8.73); % chessboard dimension in mm
square=squareSize/1000; % definition in m

ws_check=generateCheckerboardPoints(L_boardSize, squareSize);

ws_check(:,3)=zeros();

% figure(3)
% for i=1:size(ws_check,1)
%     scatter3(ws_check(i,1),ws_check(i,2),ws_check(i,3),'k','filled');
%     grid on
%     hold on
% end

% Plotting together the points in the /ws (filled) and the transformed from
% the camera (stars)
figure(4)
for i=1:size(ws_check,1)
    scatter3(ws_check(i,1),ws_check(i,2),ws_check(i,3),'k','filled');
    grid on
    hold on
    scatter3(ws_check_cam(1,i),ws_check_cam(2,i),ws_check_cam(3,i),'k*');
end

%%
figure(5)
title('everything is express in mm')
grid on
hold on
for i=1:size(worldPoints_check,1)
    scatter3(worldPoints_check(i,1),worldPoints_check(i,2),worldPoints_check(i,3),'ko');
    hold on
    scatter3(ws_check_cam(1,i),ws_check_cam(2,i),ws_check_cam(3,i),'b*');
    hold on
    scatter3(ws_check(i,1),ws_check(i,2),ws_check(i,3),'k','filled');
    pause(0.01)
end
legend('triangulate points','T-cam-pos applied','real point /ws')
hold off

%%
% Defining the transgormation coming from the error 
[regParams,Bfit,ErrorStats]=absor(ws_check',ws_check_cam(1:3,:)); 
T_err= regParams.M;

% apply to the error
ws_check_cam_err=inv(T_err)* ws_check_cam;

figure(6)
title('everything is express in mm')
plot(L_imagePoints(:,1),L_imagePoints(:,2),'r*');
hold on
grid on
plot(R_imagePoints(:,1),R_imagePoints(:,2),'b*');
hold on
for i=1:size(worldPoints_check,1)
    scatter3(worldPoints_check(i,1),worldPoints_check(i,2),worldPoints_check(i,3),'ko');
    hold on
    scatter3(ws_check_cam(1,i),ws_check_cam(2,i),ws_check_cam(3,i),'k*');
    hold on
    scatter3(ws_check(i,1),ws_check(i,2),ws_check(i,3),'k','filled');
end
scatter3(ws_check_cam_err(1,:),ws_check_cam_err(2,:),ws_check_cam_err(3,:),'c*');
legend('point L-cam','points R-cam','triangulate points','T-cam-pos applied','real point /ws','error correction T-err')
hold off




%%
ws_check=ws_check';
ws_check=ws_check./1000;

% We have to invert x and y cause they are swaped 
a=ws_check(2,:);
ws_check(2,:)=ws_check(1,:);
ws_check(1,:)=a;





%% Acquisition of the checkboard points - manual selection of the points 
for d=1
% myfig=figure(3);
% hold on
% 
% na=1; % number of acquisition of the cc to be mediated
% %% RIGHT CAMERA 
% for k=1:na
% % Right camera
%     for j=1:1%size(frame_st,2)
% 
% 
%         imshow(R_cam);
%         cursorobj = datacursormode(myfig);
% 
%         cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
%         while ~waitforbuttonpress 
%             % waitforbuttonpress returns 0 with click, 1 with key press
%             % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
%             cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
%         end
% 
%         cursorobj.Enable = 'off';
% 
%         mypoints(j) = getCursorInfo(cursorobj);
% 
%         cc_R(:,j,k)=mypoints(j).Position;
%     end
% 
% end
% 
% %%
% for k=1:na
%     % Left camera
%     for j=1:1%size(frame_st,2)
% 
% 
%         imshow(L_cam);
%         cursorobj = datacursormode(myfig);
% 
%         cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
%         while ~waitforbuttonpress 
%             % waitforbuttonpress returns 0 with click, 1 with key press
%             % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
%             cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
%         end
% 
%         cursorobj.Enable = 'off';
% 
%         mypoints(j) = getCursorInfo(cursorobj);
% 
%         cc_L(:,j,k)=mypoints(j).Position;
%     end
% 
% end 

end
%%


figure()
imshow(L_cam)
hold on
plot(cc_L(1,1),cc_L(2,1),'r*')

figure()
imshow(R_cam)
hold on
plot(cc_R(1,1),cc_R(2,1),'b*')

% PL= [501, 122];
% PR=[585, 114];
PL= [451, 200];
PR=[527, 188];

PUNTO=triangulate(cc_L',cc_R',stereoParams_new);
PUNTO_h= PUNTO';
PUNTO_h(4,1)=1;

PUNTO_ws = inv(T_cam_pose)*PUNTO_h;
PUNTO_rc= T_ws_to_rc*PUNTO_ws;

PUNTO_mov = PUNTO_rc(1:3)./1000;

move_translation(r,PUNTO_mov')
