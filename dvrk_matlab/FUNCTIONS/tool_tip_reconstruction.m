%% EDIT VIDEO AND TOOL TIP RECONSTRUCTION (triangulate)

% In order to be able to upload the video on matlab i should uncompress
% with ffmpeg.
% ffmpeg -i cali.avi -c:v libx264 -crf 10 cali_st.avi   (into the terminal)
%
% Then i can upload the video
%% Uploading the movie
v = VideoReader('motion_PSM.avi');
numberOfFrames=v.NumberOfFrames;
vidHeight = v.Height;
vidWidth = v.Width;
n=numberOfFrames;

        %% Cutting frames  ->   Bettere way to cut frames with ffmpeg -> ffmpeg -i input.mp4 -qscale:v 2 output_%03d.jpg
        for i = 1:n
         frames = read(v,i);
         imwrite(frames,['Image' int2str(i), '.jpg']);
         im(i)=image(frames);
        end
        %% Cutting frames
        % Open all the frames and create image Left and Right for each frames of interest.
        frame_st=[175 190 218 255 290 325]; % Selection of frames with to tool tip positiones 
        for i=1:n
            I(:,:,:,i)=imread(['Output_' int2str(frame_st(i)), '.jpg']);
            I_left(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
            imwrite(I_left(:,:,:,i),['Image_L' int2str(i), '.jpg']);
            I_right(:,:,:,i)=imcrop(I(:,:,:,i),[736 0 1456 vidHeight]);
            imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
        end
        %% Checking the dimension of the frames (RIGHT ONE )
        for i=1:size(frame_st,2)
            I(:,:,:,i)=imread(['Image_R' int2str(i), '.jpg']);
            I_right(:,:,:,i)=imcrop(I(:,:,:,i),[0 0 720 vidHeight]);
            imwrite(I_right(:,:,:,i),['Image_R' int2str(i), '.jpg']);
        end
        %% Undistort image
        cameraParams_R=stereoParams.CameraParameters2;
        cameraParams_L=stereoParams.CameraParameters1;
        
        for i=1:n
            I(:,:,:,i)=imread(['Image_L' int2str(i), '.jpg']);
            I_left_un(:,:,:,i) = undistortImage(I(:,:,:,i),cameraParams_L);
            imwrite(I_left_un(:,:,:,i),['Image_L_U' int2str(i), '.jpg']);
            I(:,:,:,i)=imread(['Image_R' int2str(i), '.jpg']);
            I_right_un(:,:,:,i) = undistortImage(I(:,:,:,i),cameraParams_R);
            imwrite(I_right_un(:,:,:,i),['Image_R_U' int2str(i), '.jpg']);
        end


%% SELECTION OF THE TOOL TIP

myfig=figure(3);
hold on

%% Selection of the frames where to get tip coordinates 
na=1; % number of acquisition of the cc to be mediated
%% RIGHT CAMERA 
for k=1:na
% Right camera
    for j=1:1%size(frame_st,2)


        imshow(['Image_R_U' int2str(j), '.jpg']);
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
%% LEFT CAMERA 

for k=1:na
    % Left camera
    for j=1:1%size(frame_st,2)


        imshow(['Image_L_U' int2str(j), '.jpg']);
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

%% Plotting the coordinates of the points - IMAGE PLANE
figure(4)
plot3(cc_tip_mean_L(1,:),cc_tip_mean_L(2,:),zeros(length(cc_tip_mean_L)),'r*')
hold on
grid on
plot3(cc_tip_mean_R(1,:),cc_tip_mean_R(2,:),zeros(length(cc_tip_mean_R)),'b*')
axis equal

%% 3D reconstruction - point in mm
worldPoints=triangulate(cc_tip_mean_L',cc_tip_mean_R',stereoParams);
%% Plotting the 3D reconstructed points 
worldPoints=worldPoints';
%%
figure(1)
plot3(worldPoints(1,:),worldPoints(2,:),worldPoints(3,:),'r*')
grid on

% triangulate function gives the points in respect of the optical centre (referred to the first camera),
% so coordinate are already expressed in mm.





