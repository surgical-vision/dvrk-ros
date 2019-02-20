%% Video processing for TIP coordinate acquisition 

close all 
clear all
clc
%%
% In order to be able to upload the video on matlab i should uncompress
% with ffmpeg.
% ffmpeg -i cali.avi -c:v libx264 -crf 10 cali_st.avi   (into the terminal)
%
% Then i can upload the video

v = VideoReader('stereo.avi');
numberOfFrames=v.NumberOfFrames;
vidHeight = v.Height;
vidWidth = v.Width;
n=numberOfFrames;

%% Extract all the frame related to the video
for i = 1:n
 frames = read(v,i);
 imwrite(frames,['Image' int2str(i), '.jpg']);
 im(i)=image(frames);
end

%% How to cut them
% Open all the frames and create image Left and Right for each frames.
for i=1:n
    I(:,:,:,i)=imread(['Image' int2str(i), '.jpg']);
    I_left(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 (vidWidth/2) vidHeight]);
    imwrite(I_left(:,:,:,i),['Image_L' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[(vidWidth/2) 0 vidWidth vidHeight]);
    imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
end


%%
myfig=figure(2);
hold on

%% Selection of the frames where to get tip coordinates 

imma=[7 138 152 188 202 222 254 278 298 314 380]; % selection of the number of frames to be analyzed 
lengthg=size(imma,2);

na=5; % number of acquisition of the cc to be mediated
%%
for k=1:na
% Right camera
    for j=1:lengthg


        imshow(['Image_R' int2str(imma(j)), '.jpg']);
        cursorobj = datacursormode(myfig);

        cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

        cursorobj.Enable = 'off';

        mypoints(j) = getCursorInfo(cursorobj);

        cc_tip_R(:,j,k)=mypoints(j).Position;
    end

end
%%

for k=1:na
    % Left camera
    for j=1:lengthg


        imshow(['Image_L' int2str(imma(j)), '.jpg']);
        cursorobj = datacursormode(myfig);

        cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

        cursorobj.Enable = 'off';

        mypoints(j) = getCursorInfo(cursorobj);

        cc_tip_L(:,j,k)=mypoints(j).Position;
    end

end 

%% Creaing the avarage

% Right 
for i=1:2
    for j=1:lengthg
 
    cc_tip_mean_R(i,j)=mean(cc_tip_R(i,j,:));
    
    end
end

% Left 
for i=1:2
    for j=1:lengthg
 
    cc_tip_mean_L(i,j)=mean(cc_tip_L(i,j,:));
    
    end
end

%% Plotting the coordinates of the points 
figure(3)
plot3(cc_tip_mean_L(1,:),cc_tip_mean_L(2,:),zeros(length(cc_tip_mean_L)),'r*')
hold on
grid on
plot3(cc_tip_mean_R(1,:),cc_tip_mean_R(2,:),zeros(length(cc_tip_mean_R)),'b*')

% % if you want to plot even all the other aquisition
% for i=1:na
%     
%     figure(3)
%     hold on
%     plot3(cc_tip_L(1,:,i),cc_tip_L(2,:,i),zeros(length),'r*')
%     hold on
%     plot3(cc_tip_R(1,:,i),cc_tip_R(2,:,i),zeros(length),'b*')
%     
% end 
% 



%% STEREO CALIBRATION - PROCESSING
% In order to be able to upload the video on matlab i should uncompress
% with ffmpeg.
% ffmpeg -i cali.avi -c:v libx264 -crf 10 cali_st.avi       
% In order to have a video with a good quality
% Then i can upload the video

v = VideoReader('cali_st.avi');
numberOfFrames=v.NumberOfFrames;
vidHeight = v.Height;
vidWidth = v.Width;
n=numberOfFrames;

%% Extract all the frame related to the video
for i = 1:n
 frames = read(v,i);
 imwrite(frames,['Image' int2str(i), '.jpg']);
 im(i)=image(frames);
end
%% How to cut them
% Open all the frames and create image Left and Right for each frames.
cali_st=[8 11 14 20 52 67 109 120 177 254 297 343 383 477 518 535 581 609 659 707];
for i=1:n
    I(:,:,:,i)=imread(['Image' int2str(cali_st(i)), '.jpg']);
    I_left(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
    imwrite(I_left(:,:,:,i),['Image_L' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[736 0 1456 vidHeight]);
    imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
end

%%
% Because in the R camera have different dimension
for i=1:size(cali_st,2)
    I(:,:,:,i)=imread(['Image_R' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
    imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
end

%% OPEN THE TOOLBOX FOR STEREOCALIBRATION AND FOLLOW THE INSTRUCTION
% Saving the parameter obtained by the calibration
load('estimationErrors');
load('stereoParams');

%% 3D reconstruction
% Selection of the coordinate of the tip in L/R frames
% ffmpeg -i cali.avi -c:v libx264 -crf 10 cali_st.avi  
% for encoding the video

v = VideoReader('motion.avi');
numberOfFrames=v.NumberOfFrames;
vidHeight = v.Height;
vidWidth = v.Width;
n=numberOfFrames;

%% Cutting frames
for i = 1:n
 frames = read(v,i);
 imwrite(frames,['Image' int2str(i), '.jpg']);
 im(i)=image(frames);
end
%% How to cut them
% Open all the frames and create image Left and Right for each frames of interest.
frame_st=[71 104 140 182 223 255];
for i=1:n
    I(:,:,:,i)=imread(['Image' int2str(frame_st(i)), '.jpg']);
    I_left(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
    imwrite(I_left(:,:,:,i),['Image_L' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[736 0 1456 vidHeight]);
    imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
end
%%
for i=1:size(frame_st,2)
    I(:,:,:,i)=imread(['Image_R' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
    imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
end


%% SELECTION OF THE TOOL TIP
myfig=figure(3);
hold on

%% Selection of the frames where to get tip coordinates 

na=5; % number of acquisition of the cc to be mediated
%%
for k=1:na
% Right camera
    for j=1:size(frame_st,2)


        imshow(['Image_R' int2str(j), '.jpg']);
        cursorobj = datacursormode(myfig);

        cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

        cursorobj.Enable = 'off';

        mypoints(j) = getCursorInfo(cursorobj);

        cc_tip_R(:,j,k)=mypoints(j).Position;
    end

end
%%

for k=1:na
    % Left camera
    for j=1:size(frame_st,2)


        imshow(['Image_L' int2str(j), '.jpg']);
        cursorobj = datacursormode(myfig);

        cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

        cursorobj.Enable = 'off';

        mypoints(j) = getCursorInfo(cursorobj);

        cc_tip_L(:,j,k)=mypoints(j).Position;
    end

end 

%% Creaing the avarage

% Right 
for i=1:2
    for j=1:size(frame_st,2)
 
    cc_tip_mean_R(i,j)=mean(cc_tip_R(i,j,:));
    
    end
end

% Left 
for i=1:2
    for j=1:size(frame_st,2)
 
    cc_tip_mean_L(i,j)=mean(cc_tip_L(i,j,:));
    
    end
end

%% Plotting the coordinates of the points 
figure(4)
plot3(cc_tip_mean_L(1,:),cc_tip_mean_L(2,:),zeros(length(cc_tip_mean_L)),'r*')
hold on
grid on
plot3(cc_tip_mean_R(1,:),cc_tip_mean_R(2,:),zeros(length(cc_tip_mean_R)),'b*')

%% 3D reconstruction
worldPoints=triangulate(cc_tip_mean_L',cc_tip_mean_R',stereoParams);









%% Exstrinsic calibration of LEFT camera
cameraParams_L=stereoParams.CameraParameters1;

%% Selectin the frame for estrinsic calibration
v = VideoReader('estrinsic.avi');
for i = 1:5
 frames = read(v,i);
 imwrite(frames,['Image' int2str(i), '.jpg']);
 im(i)=image(frames);
end
%%
I(:,:,:,1)=imread(['Image' int2str(1), '.jpg']);
I_left(:,:,:,1)=imcrop(I(:,:,:,1),[0 0 720 576]);
imwrite(I_left(:,:,:,1),['Image_L' int2str(1), '.jpg']);
I_right(:,:,:,1)=imcrop(I(:,:,:,1),[736 0 1456 546]);
imwrite(I_right(:,:,:,1),['Image_R' int2str(1), '.jpg']);

%%
im = imread('Image_L1.jpg');

[im] = undistortImage(im,cameraParams_L,'OutputView','full');
%%

[imagePoints_,boardSize_] = detectCheckerboardPoints(im);
imagePoints = imagePoints_(:,:);       

figure();
imshow(im); hold 'on';
for idx = 1:size(imagePoints,1)
   plot(imagePoints(idx,1),imagePoints(idx,2),'go'); hold 'on';
   pause(1)
end

%% change square size if necessary
squareSize = 10;
boardSize = [ boardSize_(1) , boardSize_(2) ] ;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
figure()
plot(worldPoints(:,1),worldPoints(:,2));
%%
[rotationMatrix,translationVector] = extrinsics(imagePoints,worldPoints,cameraParams_L);
rotationMatrix
translationVector

%%
[orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
  translationVector);
figure
plotCamera('Location',location,'Orientation',orientation,'Size',4);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
  'VerticalAxisDir','down','MarkerSize',40);


figure()
imshow(im); hold 'on';
imgPts = worldToImage(cameraParams_L, rotationMatrix, translationVector, [0,0,0]);
plot( imgPts(1),imgPts(2),'g*');







