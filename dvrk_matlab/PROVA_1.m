%% Subscribing image coming from the calibation node 

image_sub=rossubscriber('/image_output/left/image_rect','sensor_msgs/Image');
msgImg = receive(image_sub);
L_cam= readImage(msgImg);



image_sub_raw=rossubscriber('/image_output/left/image_mono','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_raw= readImage(msgImg_raw);