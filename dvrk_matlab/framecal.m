%% Extraction of the frames for calibration 
close all
clear all
clc
%%
% In order to be able to upload the video on matlab i should uncompress
% with ffmpeg.
% ffmpeg -i input.avi -an -vcodec rawvideo -y output.avi  (into the terminal)
%
% Then i can upload the video

v = VideoReader('call.avi');
numberOfFrames=v.NumberOfFrames;
vidHeight = v.Height;
vidWidth = v.Width;
n=numberOfFrames;

%% Extract all the frame related to the video
for i = 1:n
 frames = read(v,i);
 imwrite(frames,['Frame' int2str(i), '.jpg']);
 im(i)=image(frames);
end

%% How to cut them
% Open all the frames and create image Left and Right for each frames.
for i=1:n
    I(:,:,:,i)=imread(['Frame' int2str(i), '.jpg']);
    I_left(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 (vidWidth/2) vidHeight]);
    imwrite(I_left(:,:,:,i),['Frame_L' int2str(i), '.jpg']);
    I_right(:,:,:,i)=imcrop(I(:,:,:,i),[(vidWidth/2) 0 vidWidth vidHeight]);
    imwrite(I_right(:,:,:,i),['Frame_R' int2str(i), '.jpg']);
end


%% Definition of the intrinsic calibration

Intrinsic_L=cameraParams.IntrinsicMatrix;




