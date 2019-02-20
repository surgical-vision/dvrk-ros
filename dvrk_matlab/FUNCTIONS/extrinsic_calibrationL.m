%% EXTRINSIC CAMERA CALIBRATION 
close all 
clear all
clc

%% Exstrinsic calibration of LEFT camera    -   after stereo calibration
cameraParams_L=stereoParams.CameraParameters1;

%% Selection of the frames from ROS - saving them from the launch file 

im = imread('ee.png');
squareSize=10;
intrinsicParam=cameraParams_L;
%%
[rotationMatrix,translationVector,Text]=extrinsic_calibration(im,intrinsicParam,squareSize);

%% Selectin the frame for estrinsic calibration
% % Just one frame needed 
% 
% v = VideoReader('extrinsic.avi');
% for i = 1:2
%  frames = read(v,i);
%  imwrite(frames,['Image' int2str(i), '.jpg']);
%  im(i)=image(frames);
% end
% %% Cuttin frames 
% I(:,:,:,1)=imread(['Image' int2str(1), '.jpg']);
% I_left(:,:,:,1)=imcrop(I(:,:,:,1),[0 0 720 576]);
% imwrite(I_left(:,:,:,1),['Image_L' int2str(1), '.jpg']);
% I_right(:,:,:,1)=imcrop(I(:,:,:,1),[736 0 1456 546]);
% imwrite(I_right(:,:,:,1),['Image_R' int2str(1), '.jpg']);
%% Selectin the frame and computing the calibration
% im = imread('Image_L1.jpg');
% squareSize=10;
% intrinsicParam=cameraParams_L;
% [rotationMatrix,translationVector,Text]=extrinsic_calibration(im,intrinsicParam,squareSize);

%% Checking the accuracy calibration plotting a point 

% %%
im = imread('ee.png');
[im] = undistortImage(im,cameraParams_L);
for d=1
%
% 
% [imagePoints_,boardSize_] = detectCheckerboardPoints(im);
% imagePoints = imagePoints_(:,:);       
% 
% figure();
% imshow(im); hold 'on';
% for idx = 1:size(imagePoints,1)
%    plot(imagePoints(idx,1),imagePoints(idx,2),'go'); hold 'on';
%    pause(1)
% end
% 
% %% change square size if necessary
% squareSize = 10;
% boardSize = [ boardSize_(1) , boardSize_(2) ] ;
% worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% figure()
% plot(worldPoints(:,1),worldPoints(:,2));
% %%
% [rotationMatrix,translationVector] = extrinsics(imagePoints,worldPoints,cameraParams_L);
% rotationMatrix
% translationVectorimagePoints
% 
% %%imagePoints_mean(:,:)=mean(imagePoints,3);
boardSize = [ squaredim(1,1)+1 , squaredim(1,2)+1 ] ;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
[rotationMatrix,translationVector] = extrinsics(imagePoints_mean,worldPoints,intrinsicParam);

Text=zeros(4);
Text(1:3,1:3)=rotationMatrix;
Text(1:3,4)=translationVector;
Text(4,4)=1;
% [orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
%   translationVector);
% figure
% plotCamera('Location',location,'Orientation',orientation,'Size',4);
% hold on
% pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
%   'VerticalAxisDir','down','MarkerSize',40);
% 
% 
end


figure(55)
imshow(im); hold 'on';imagePoints_mean(:,:)=mean(imagePoints,3);
boardSize = [ squaredim(1,1)+1 , squaredim(1,2)+1 ] ;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
[rotationMatrix,translationVector] = extrinsics(imagePoints_mean,worldPoints,intrinsicParam);

Text=zeros(4);
Text(1:3,1:3)=rotationMatrix;
Text(1:3,4)=translationVector;
Text(4,4)=1;
imgPts = worldToImage(cameraParams_L, rotationMatrix, translationVector, [1,1,0; 2,1,0]);
plot( imgPts(1),imgPts(2),'g*');

%% Testing the accuracy of this transformation 







