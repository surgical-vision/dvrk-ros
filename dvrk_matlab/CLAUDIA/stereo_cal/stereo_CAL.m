%% For STEREO CALIBRATION acquiring frames subrscibing the ron node
 
% rosinit
% rosshutdown
 
%% Acquisition of frames 
   
 
 
frame=1;
  
while(frame < 31)
    
    L_image_sub_raw=rossubscriber('/image_output/left/image_mono','sensor_msgs/Image');
    L_msgImg_raw = receive(L_image_sub_raw);
    L_cam_grey(:,:,frame)= readImage(L_msgImg_raw);
    figure(frame)
    subplot(1,2,1)
    imshow(L_cam_grey(:,:,frame))
 
    R_image_sub_raw=rossubscriber('/image_output/right/image_mono','sensor_msgs/Image');
    R_msgImg_raw = receive(R_image_sub_raw);
    R_cam_grey(:,:,frame)= readImage(R_msgImg_raw);
    subplot(1,2,2)
    imshow(R_cam_grey(:,:,frame))
    
    imwrite(L_cam_grey(:,:,frame),sprintf('LEFT-%d.png',frame));
    imwrite(R_cam_grey(:,:,frame),sprintf('RIGHT-%d.png',frame));
 
    while ~waitforbuttonpress 
                    
    end                             
      
      frame=frame+1;
      close
end
 
%%
L_msgImg_raw = receive(L_image_sub_raw);
L_cam_grey= readImage(L_msgImg_raw);
 
R_msgImg_raw = receive(R_image_sub_raw);
R_cam_grey= readImage(R_msgImg_raw);

