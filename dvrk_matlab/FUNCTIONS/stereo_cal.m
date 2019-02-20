%% STEREO CALIBRATION OF ENDOSCOPE 
close all
clear cll
clc

%% 
% In order to be able to upload the video on matlab i should uncompress
% with ffmpeg.
% ffmpeg -i cali.avi -c:v libx264 -crf 10 cali_st.avi       
% In order to have a video with a good quality
% Then i can upload the video

v = VideoReader('cali_st.avi');
% v = VideoReader('stereoL.avi');
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
cali_st=[8 11 14 20 52 67 109 120 177 254 297 343 383 477 518 535 581 609 659 707];      %selecting around 20 frames for calibration 
for i=1:n
    I(:,:,:,i)=imread(['Image' int2str(cali_st(i)), '.jpg']);
    I_left(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
    imwrite(I_left(:,:,:,i),['Image_L' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[736 0 1456 vidHeight]);
    imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
end

%% Checking the obtained dimension of frames 
% Because in the R camera have different dimension
for i=1:size(cali_st,2)
    I(:,:,:,i)=imread(['Image_R' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
    imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
end

%% OPEN THE TOOLBOX FOR STEREOCALIBRATION AND FOLLOW THE INSTRUCTION
% Saving the parameter obtained by the calibration
% You have to save the variables from the workspace
load('estimationErrors');
load('stereoParams');
