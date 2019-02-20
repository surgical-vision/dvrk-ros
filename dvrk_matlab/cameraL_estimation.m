%% Acquired image coming from the endoscope
 
rosinit     % initialise the bridge with ROS
% rosshutdown    % Shut down the connection with ros network
 
%% Loading stereo parameters
load('stereoParams.mat')
camera1=stereoParams_new.CameraParameters1;
%% Acquiring the frame: un-comment according to the type of image needed [576x720]
% Creation of a subscriber to the ROS node that publishes the left frame
% Rememeber to save the frame if needed
 
% Acquisition of a gre-scale image  
image_sub_raw=rossubscriber('/image_output/left/image_mono','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey= readImage(msgImg_raw);
L_cam = undistortImage(L_cam_grey,camera1);

% figure(3)
% imshow(L_cam);
% hold on;
% plot(imagePonints(:,1),imagePonints(:,2),'r*');
% hold on;
% plot(imagePoints_back_T(1,:),imagePoints_back_T(2,:),'b*');
 
% % Acquisition of rbg image
% image_sub_raw=rossubscriber('/image_output/left/image_color','sensor_msgs/Image');
% msgImg_raw = receive(image_sub_raw);
% L_cam_color= readImage(msgImg_raw);
% L_cam = undistortImage(L_cam_color,camera1);
 
% % Acquisition of a rectified image.
% image_sub_raw=rossubscriber('/image_output/left/image_rect','sensor_msgs/Image');
% msgImg_raw = receive(image_sub_raw);
% L_cam_rect= readImage(msgImg_raw);
% L_cam = undistortImage(L_cam_rect,camera1);
 
%% Detecting checkborad  
 
[imagePonints, boardSize]= detectCheckerboardPoints(L_cam);
 
figure(1);
imshow(L_cam);%% For STEREO CALIBRATION acquiring frames subrscibing the ron node
hold on;
plot(imagePonints(:,1),imagePonints(:,2),'r*');
 
%% Definition of the worldPoints
 
dim= 8.73;  %square dimension in mm
len = boardSize(1,2)-1;
hig = boardSize(1,1)-1;
 
% Considering the dimentsion as follow 
% 
% --------------> len
% |
% |
% |
% | 
% hig
k=1;
for i=1:hig
    for j=1:len
        
        worldPoints(k,1:3)=[j*dim i*dim 0*dim];
        k=k+1;
        
    end
end
 
%% Definition of camera orientation
[worldOrientation, worldLocation]= estimateWorldCameraPose(imagePonints,worldPoints,camera1);
 
%% Plotting 
figure(2)
 pcshow(worldPoints,'VerticalAxis','Y','VerticalAxisDir','down', ...
     'MarkerSize',30);
 hold on
 plotCamera('Size',10,'Orientation',worldOrientation,'Location',...
     worldLocation);
 hold off
 
 %% Checking with the back projection of the points
T_cam(1:3,1:3)=worldOrientation;
T_cam(1:3,4)= worldLocation;
T_cam(4,:)= [0 0 0 1];
 
worldPoints_h = worldPoints;
worldPoints_h(:,4)= ones;

imagePoints_back_T=T_cam*worldPoints_h';

% for=1:size(worldPoints_h,1)
%     imagePoints_back_T(i,:)=T_cam*worldPoints_h;
% end 
 
imagePoints_back= worldToImage(camera1,worldOrientation,worldLocation,worldPoints);
 
figure(3)
imshow(L_cam);
hold on;
plot(imagePonints(:,1),imagePonints(:,2),'r*');
hold on;
plot(imagePoints_back_T(1,:),imagePoints_back_T(2,:),'b*');
hold on;
plot(imagePoints_back(1,:),imagePoints_back(2,:),'g*');
 
 