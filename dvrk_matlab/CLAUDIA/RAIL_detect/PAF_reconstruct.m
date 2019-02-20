%% RAIL detection

image_sub_raw=rossubscriber('/image_output/left/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'LEFT-PAF1.png');

image_sub_raw=rossubscriber('/image_output/right/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'RIGHT-PAF1.png');

%%
% Image processing 
load('stereoParamsD.mat');
camera1= stereoParamsD.CameraParameters1;
camera2= stereoParamsD.CameraParameters2;

L_cam_grey= imread('LEFT-PAF1.png');
L_cam = undistortImage(L_cam_grey, camera1);

R_cam_grey= imread('RIGHT-PAF1.png');
R_cam = undistortImage(R_cam_grey, camera2);


squareSize=1.03; % in mm

% detect 2D checkerboard points
[ L_imagePoints , L_boardSize ] = detectCheckerboardPoints(L_cam) ;
[ R_imagePoints , R_boardSize ] = detectCheckerboardPoints(R_cam) ;



% Let's show how they are detected
figure()
hold on;
imshow(L_cam)
hold on
for i=1:size( L_imagePoints,1)
   
    plot(L_imagePoints(i,1),L_imagePoints(i,2),'r*')
    pause(0.3)
    
end
pause(3)
%close
% Let's show how they are detected
figure()
hold on;
imshow(R_cam)
hold on
for i=1:size( R_imagePoints,1)
   
    plot(R_imagePoints(i,1),R_imagePoints(i,2),'b*')
    pause(0.3)
    
end
close
pause(3)
close
% plotting the point in the same picture 
figure(11)
grid on
hold on
for i=1:size( R_imagePoints,1)

plot(R_imagePoints(i,1),R_imagePoints(i,2),'b*')
hold on
plot(L_imagePoints(i,1),L_imagePoints(i,2),'r*')
pause(0.1)
end

% reconstruct the point in the camera pose
worldPoints_PAF=triangulate(L_imagePoints,R_imagePoints,stereoParamsD);
worldPoints_PAF=worldPoints_PAF./1000;

figure()
hold on
grid on
for i=1:size( R_imagePoints,1)
    hold on
    scatter3(worldPoints_PAF(i,1),worldPoints_PAF(i,2),worldPoints_PAF(i,3),'k','filled')
    pause(0.05)
end

%
squaret=squareSize/1000;
check_tiny=generateCheckerboardPoints(L_boardSize, squaret);

check_tiny(:,3)=zeros();

figure(3)
hold on
for i=1:size(check_tiny,1)
    scatter3(check_tiny(i,1),check_tiny(i,2),check_tiny(i,3),'k','filled');
    grid on
    hold on
    scatter3(worldPoints_PAF(i,1),worldPoints_PAF(i,2),worldPoints_PAF(i,3),'k*')
    pause(0.01)
end
%
[regParams,Bfit,ErrorStats]=absor(worldPoints_PAF(:,:)',check_tiny(:,:)'); 
T_tiny= regParams.M;

check_tiny_h=check_tiny';
check_tiny_h(4,:)=ones();
check_tiny_back= inv(T_tiny)* check_tiny_h;

worldPoints_PAF_h=worldPoints_PAF';
worldPoints_PAF_h(4,:)=ones();
worldPoints_PAF_back= T_tiny* worldPoints_PAF_h;

% Creation of RF
RF_ws=createRF([1*squaret;0;0],[4*squaret;0;0],[0;2*squaret;0;],[0;0;0]);

figure(3)
hold on
% trplot(RF_ws,'length',20,'color','r');    % mm
 trplot(RF_ws,'length',0.02,'color','k');  % meter

RF_3D_point=inv(T_tiny)*RF_ws;%%
[regParams,Bfit,ErrorStats]=absor(worldPoints_PAF_back(1:3,:),check_tiny(:,:)'); 
T_scale= regParams.M;

worldPoints_PAF_back_back= T_scale* worldPoints_PAF_back;

figure(3)
hold on
scatter3(worldPoints_PAF_back(1,:),worldPoints_PAF_back(2,:),worldPoints_PAF_back(3,:),'co','filled')

figure(3)
hold on
trplot(RF_3D_point,'length',0.02,'color','r')
 
figure(3)
hold on
scatter3(check_tiny_back(1,:),check_tiny_back(2,:),check_tiny_back(3,:),'ro')
hold on
scatter3(worldPoints_PAF_back(1,:),worldPoints_PAF_back(2,:),worldPoints_PAF_back(3,:),'bo','filled')
% hold on
% scatter3(worldPoints_PAF_back(1,1).*squaret,worldPoints_PAF_back(2,1).*squaret,worldPoints_PAF_back(3,1).*squaret,'b*')


%%
GRASP_p=check_tiny(15,:);
GRASP_p(1,2)=GRASP_p(1,2) + 0.051; % summing in m 
GRASP_p_h=GRASP_p';
GRASP_p_h(4,1)=1;

GRASP_p_back= inv(T_tiny)* GRASP_p_h;

figure(3)
hold on
scatter3(GRASP_p(1,1),GRASP_p(1,2),GRASP_p(1,3),'k','filled')
hold on
scatter3(GRASP_p_back(1,1),GRASP_p_back(2,1),GRASP_p_back(3,1),'k*')
%
worldPoints_PAF(25,1:3)= GRASP_p_back(1:3,1)';
%%
%Transforming point in the ws reference frame 
worldPoints_PAF=worldPoints_PAF';
PAF_Points_h=worldPoints_PAF;
PAF_Points_h(4,:)=ones();

PAF_Points_ws= inv(T_err_mm)*PAF_Points_h;

% To show in m
figure()
hold on
scatter3(PAF_Points_ws(1,:),PAF_Points_ws(2,:),PAF_Points_ws(3,:),'c','filled')
grid on

% Generating motion to checkboard 
for i=1:size(PAF_Points_ws,2)
    
    PAF_Points_move_psm(:,i)=T_ws_to_rc*PAF_Points_ws(:,i);

end

figure()
hold on
scatter3(PAF_Points_move_psm(1,:),PAF_Points_move_psm(2,:),PAF_Points_move_psm(3,:),'r','filled')
grid on

%
a=PAF_Points_move_psm(1:3,25)';
%%
move_translation(r,a)






