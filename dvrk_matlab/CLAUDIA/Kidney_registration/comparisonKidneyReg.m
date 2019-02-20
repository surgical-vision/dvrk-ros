%% COMPARISON IN KIDNEY REGISTRATION 

%%  METHOD 1 - ENDOSCOPIC MARKER TRACKING  
%Image acquisition - Acquisition of a grey-scale image  

image_sub_raw=rossubscriber('/image_output/left/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'LEFT-COLO1.png');

image_sub_raw=rossubscriber('/image_output/right/image_color','sensor_msgs/Image');
msgImg_raw = receive(image_sub_raw);
L_cam_grey_ROS= readImage(msgImg_raw);
imwrite(L_cam_grey_ROS,'RIGHT-COLO1.png');

% Image processing 
load('stereoParams_new.mat');
intri1= stereoParams_new.CameraParameters1;
intri2= stereoParams_new.CameraParameters2;

% Undistort the image 
IL= imread('LEFT-MONO1.png');
IR= imread('RIGHT-MONO1.png');
ILu = undistortImage(IL,intri1);
IRu = undistortImage(IR,intri2);

figure()
imshow(ILu)

% figure()
% imshow(IL)

figure()
imshow(IRu)

% figure()
% imshow(IR)
%%
% Definition of the marker threshold 
ILu_mask = roicolor(ILu,20,38);
IRu_mask = roicolor(IRu,20,38);


figure()
subplot(1,2,1)
imshow(ILu_mask)
subplot(1,2,2)
imshow(IRu_mask)

%% Selecting the points - manually 

myfig=figure(30);
hold on
%% LEFT CAMERA 

% Left camera
for j=1:4 %size(frame_st,2)


    imshow(ILu);
    cursorobj = datacursormode(myfig);

    cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

    cursorobj.Enable = 'off';

    mypoints(j) = getCursorInfo(cursorobj);

    cc_dot_L(:,j)=mypoints(j).Position;
end


%% RIGHT CAMERA
 % right camera
for j=1:4%size(frame_st,2)


    imshow(IRu);
    cursorobj = datacursormode(myfig);

    cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
        while ~waitforbuttonpress 
            % waitforbuttonpress returns 0 with click, 1 with key press
            % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
            cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
        end

    cursorobj.Enable = 'off';

    mypoints(j) = getCursorInfo(cursorobj);

    cc_dot_R(:,j)=mypoints(j).Position;
end


% The trinagulate function reconstruct the point respect to the optical
% centre of the first camera (left in this case) so poits are already in mm
kidPoints=triangulate(cc_dot_L',cc_dot_R',stereoParams_new);

figure()
plot3(kidPoints(:,1),kidPoints(:,2),kidPoints(:,3),'r*')
grid on

%% Transforming the point in the ws reference frame 
kidPoints=kidPoints';
kidPoints_h=kidPoints;
kidPoints_h(4,:)=ones();

kidPoints_ws= inv(T_cam_pose)*kidPoints_h;
kidPoints_ws=kidPoints_ws./1000;

figure(66)
plot3(kidPoints_ws(1,:),kidPoints_ws(2,:),kidPoints_ws(3,:),'r*')
grid on


%% COMPARISON WITH TOOL TIP ACQUISITION 

count=1;
numpoint= 5; % num of points acquired for the acquisition + 1 

while (count<numpoint)
    
     c1=r.get_state_joint_current;
     cjoint_kid(:,:,count)=c1(1,1);   % generation of the matrix of joint value

     c1c=r.get_position_current();
     cc_kid(:,:,count)=c1c;    
    
    while ~waitforbuttonpress 
                    
    end                                
      count=count+1;
      close
end

np=numpoint;

for i=1:np-1
    tele_kid(:,i)=cc_kid(1:3,4,i);
end

tele_kid_h=tele_kid;
tele_kid_h(4,:)=ones();

kidPoints_nd = T_ws_to_rc * tele_kid_h;

% Plotting the results
figure(66)
hold on
plot3(kidPoints_nd(1,:),kidPoints_nd(2,:),kidPoints_nd(3,:),'b*')

%% checking the distance

norm(kidPoints_nd(1:3,1)-kidPoints_nd(1:3,2))
norm(kidPoints_ws(1:3,1)-kidPoints_ws(1:3,2))


norm(kidPoints_nd(1:3,2)-kidPoints_nd(1:3,3))
norm(kidPoints_ws(1:3,2)-kidPoints_ws(1:3,3))

norm(kidPoints_nd(1:3,3)-kidPoints_nd(1:3,4))
norm(kidPoints_ws(1:3,3)-kidPoints_ws(1:3,4))


