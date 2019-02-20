%% Definition of the pose of LEFT camera in respect of the workspace. 
% Using:
%       - triangulated point
%       - Horn Methods for matrix estimation

load('stereoParams_cla.mat')
% load('stereoParams_evs.mat')
% load('stereoParams_mirek.mat')

%% FIRST:manually selection of points
        % There is the need of having two undistorted frame for this evaluation 
%         IR = imread(['Image_R' int2str(3), '.jpg']);
                        IR_U = imread(['Image_R_U' int2str(1), '.jpg']);
%         IR_U = undistortImage(IR,stereoParams.CameraParameters2);
        %% Open the figure  - RIGHT CAMERA 
        myfig=figure(3);
        hold on
        imshow(IR_U);
        na=1;
        %% Point selectiones - RIGHT CAMERA 
        for k=1:na
        % Right camera
            for j=1:20 % CHANGE ACCORDIN THE DESIRED NUMBER OF POINTS


        %         imshow(['Image_R' int2str(1), '.jpg']);
                cursorobj = datacursormode(myfig);

                cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
                while ~waitforbuttonpress 
                    % waitforbuttonpress returns 0 with click, 1 with key press
                    % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
                    cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
                end

                cursorobj.Enable = 'off';

                mypoints(j) = getCursorInfo(cursorobj);

                cc_CAL_R(:,j,k)=mypoints(j).Position;
            end

        end
        close
        %% Selecting and undistort the image - LEFT CAMERA 
%         IL = imread(['Image_L_U' int2str(3), '.jpg']);
                                        IL_U = imread(['Image_L_U' int2str(1), '.jpg']);
%         IL_U = undistortImage(IL,stereoParams.CameraParameters1);

        %% Open the figure - LEFT CAMERA 
        myfig=figure(4);
        hold on
        imshow(IL_U)
        %% Point selectiones - LEFT  CAMERA
        for k=1:na
            % Left camera
            for j=1:20  % CHANGE ACCORDIN THE DESIRED NUMBER OF POINTS


                cursorobj = datacursormode(myfig);

                cursorobj.SnapToDataVertex = 'on'; % Snap to our plotted data, on by default
                while ~waitforbuttonpress 
                    % waitforbuttonpress returns 0 with click, 1 with key press
                    % Does not trigger on ctrl, shift, alt, caps lock, num lock, or scroll lock
                    cursorobj.Enable = 'on'; % Turn on the data cursor, hold alt to select multiple points
                end

                cursorobj.Enable = 'off';

                mypoints(j) = getCursorInfo(cursorobj);

                cc_CAL_L(:,j,k)=mypoints(j).Position;
            end

        end 
        close

        %% Rename variables
        for d=1
        cc_CAL_mean_L=cc_CAL_L;
        cc_CAL_mean_R=cc_CAL_R;
        end 
    %% Plotting the coordinates of the points just acquired - IMAGE PLANE 
    W=repmat([45],1,1);
    w=W(:);
    figure(5)
    hold on
    axis equal
    for i=1:size(cc_CAL_mean_L,2)
        scatter3(cc_CAL_mean_L(1,i),cc_CAL_mean_L(2,i),zeros(length(cc_CAL_mean_L(i))),w,'r*')
        hold on
        pause(0.1)
        grid on
        scatter3(cc_CAL_mean_R(1,i),cc_CAL_mean_R(2,i),zeros(length(cc_CAL_mean_R(i))),w,'b*')
        pause(0.1)
    end

%% SECOND: Improving corner detection
% ILG=rgb2gray(IL_U);
% IRG=rgb2gray(IR_U);
ILG=IL_U;
IRG=IR_U;

five=ones(2,20);
five=five*5;
cc_CAL_mean_L_f=cc_CAL_mean_L-five;
cc_CAL_mean_R_f=cc_CAL_mean_R-five;
        %% HARRIS FEATURES 
        point_L=zeros(size(cc_CAL_mean_L_f,2),4);
        point_R=zeros(size(cc_CAL_mean_L_f,2),4);

            for i=1:size(cc_CAL_mean_L_f,2)
                points = detectHarrisFeatures(ILG,'ROI',[cc_CAL_mean_L_f(1,i) cc_CAL_mean_L_f(2,i) 10 10]); 
                if points.Count==1
                    point_L(i,1:2)=points.Location;
                else
                    point_L(i,1:2)=points.Location(1,:);
                    point_L(i,3:4)=points.Location(2,:);
                end
                points= detectHarrisFeatures(IRG,'ROI',[cc_CAL_mean_R_f(1,i) cc_CAL_mean_R_f(2,i) 10 10]);    
                if points.Count==1
                    point_R(i,1:2)=points.Location;
                else
                    point_R(i,1:2)=points.Location(1,:);
                    point_R(i,3:4)=points.Location(2,:);
                end
            end

        %% Another detector -   MIN EIGEN FEATURES 
            for i=1:size(cc_CAL_mean_L_f,2)
                points = detectMinEigenFeatures(ILG,'ROI',[cc_CAL_mean_L_f(1,i) cc_CAL_mean_L_f(2,i) 10 10]); 
                if points.Count==1
                    point_L2(i,1:2)=points.Location;
                else
                    point_L2(i,1:2)=points.Location(1,:);
                    point_L2(i,3:4)=points.Location(2,:);
                end
                points= detectMinEigenFeatures(IRG,'ROI',[cc_CAL_mean_R_f(1,i) cc_CAL_mean_R_f(2,i) 10 10]);    
                if points.Count==1
                    point_R2(i,1:2)=points.Location;
                else
                    point_R2(i,1:2)=points.Location(1,:);
                    point_R2(i,3:4)=points.Location(2,:);
                end
            end

        %% plottin ALL the point - IMAGE PLANE 
        figure(5)
        hold on
        axis equal
        grid on
        for i=1:size(point_L,1)
            scatter3(point_L2(i,1),point_L2(i,2),zeros(length(point_L2(i))),w,'r')
            if point_L2(i,3:4)~=0
                scatter3(point_L2(i,3),point_L2(i,4),zeros(length(point_L2(i))),w,'r','x')
            end
            hold on
            scatter3(point_L(i,1),point_L(i,2),zeros(length(point_L(i))),w,'r','filled')
            if point_L(i,3:4)~=0
                scatter3(point_L(i,3),point_L(i,4),zeros(length(point_L(i))),w,'r','^')
            end
            hold on
            scatter3(point_R2(i,1),point_R2(i,2),zeros(length(point_R2(i))),w,'b')
            if point_L2(i,3:4)~=0
                scatter3(point_R2(i,3),point_R2(i,4),zeros(length(point_R2(i))),w,'b','x')
            end
            hold on
            pause(0.1)
            scatter3(point_R(i,1),point_R(i,2),zeros(length(point_R(i))),w,'b','filled')
            if point_R(i,3:4)~=0
                scatter3(point_R(i,3),point_R(i,4),zeros(length(point_R(i))),w,'b','^')
            end
            pause(0.1)
        end 

        %% Overlapping the points over the chess board  - REAL FIGURE
        figure(6)
        imshow(IL_U)
        axis equalstereoParams.CameraParameters1
        grid on

        figure(7)
        imshow(IR_U)
        axis equal
        grid on

        for i=1:size(point_L,1)
            figure(6)
            hold on
            scatter3(cc_CAL_mean_L(1,i),cc_CAL_mean_L(2,i),zeros(length(cc_CAL_mean_L(i))),w,'r*')
            hold on
            scatter3(point_L(i,1),point_L(i,2),zeros(length(point_L(i))),w,'r','filled')
            if point_L(i,3:4)~=0
                scatter3(point_L(i,3),point_L(i,4),zeros(length(point_L(i))),w,'r','^')
            end
            hold on
            scatter3(point_L2(i,1),point_L2(i,2),zeros(length(point_L2(i))),w,'r')
            if point_L2(i,3:4)~=0
                scatter3(point_L2(i,3),point_L2(i,4),zeros(length(point_L2(i))),w,'r','x')
            end
            hold on
            pause(0.1)
            figure(7)
            hold on
            scatter3(cc_CAL_mean_R(1,i),cc_CAL_mean_R(2,i),zeros(length(cc_CAL_mean_R(i))),w,'b*')
            hold on
            scatter3(point_R(i,1),point_R(i,2),zeros(length(point_R(i))),w,'b','filled')
            if point_R(i,3:4)~=0
                scatter3(point_R(i,3),point_R(i,4),zeros(length(point_R(i))),w,'b','^')
            end 
            hold on
            scatter3(point_R2(i,1),point_R2(i,2),zeros(length(point_R2(i))),w,'b')
            if point_L2(i,3:4)~=0
                scatter3(point_R2(i,3),point_R2(i,4),zeros(length(point_R2(i))),w,'b','x')
            end
            pause(0.1)
        end


%% APPLYING THE PRE-PROCESSING FUNCTR_image_sub_raw=rossubscriber('/image_output/right/image_mono','sensor_msgs/Image');
ION - optimum method
% i have to convert coordinate of point in mm
T_in_L=stereoParams.CameraParameters1.IntrinsicMatrix';
T_in_R=stereoParams.CameraParameters2.IntrinsicMatrix';

        % trasforming in homogeneus coordinate - USING POINTS MANUALLY DETECTED 
        cc_CAL_L_h=cc_CAL_mean_L;
        cc_CAL_L_h(3,:)=ones();
        cc_CAL_R_h=cc_CAL_mean_R;
        cc_CAL_R_h(3,:)=ones();

        % trasforming from pp to mm
        for i=1:size(cc_CAL_mean_L,2)
            cc_L_MM(:,i)=inv(T_in_L)*cc_CAL_L_h(:,i);
            cc_R_MM(:,i)=inv(T_in_R)*cc_CAL_R_h(:,i);

        end

        % Defining transformation matrix from 1 to 2
        T_L_to_R(1:3,1:3)=stereoParams.RotationOfCamera2;
        T_L_to_R(1:3,4)=stereoParams.TranslationOfCamera2;
        T_L_to_R(4,:)=[0 0 0 1];

        % Using the pre-processing function and trasforming everything back from mm
        % to ppR_image_sub_raw=rossubscriber('/image_output/right/image_mono','sensor_msgs/Image');

        for i=1:size(cc_L_MM,2)
            [x1, x2] = sfmTriangulationOptCond(cc_L_MM(:,i),cc_R_MM(:,i),T_L_to_R);
            cc_L_MM_opt(:,i)=x1;
            cc_R_MM_opt(:,i)=x2;
            % convert back to pixel 
            cc_L_opt_p(:,i)=T_in_L*cc_L_MM_opt(:,i);
            cc_R_opt_p(:,i)=T_in_R*cc_R_MM_opt(:,i);


        end

        cc_L_opt_p=cc_L_opt_p(1:2,:);
        cc_R_opt_p=cc_R_opt_p(1:2,:);

%% Using the triangulation function 
rot = stereoParams.RotationOfCamera2;
tra = stereoParams.TranslationOfCamera2;
camMatrix1 = cameraMatrix(stereoParams.CameraParameters1, eye(3), [0 0 0]);
camMatrix2 = cameraMatrix(stereoParams.CameraParameters2, rot, tra);
[imagePoints_cal,reprojerror]=triangulate(cc_L_opt_p(:,:)',cc_R_opt_p(:,:)',camMatrix1, camMatrix2);
% distanceInMeters = norm(imagePoints_cal)/1000;
mean(reprojerror)

%% Defining same points     -   WORLD POINT (same number of the beginning )
square_mm=8.73;
for d=1
worldPoints_cal(1,1:3)=[1*square_mm 1*square_mm 0*square_mm];
worldPoints_cal(2,1:3)=[2*square_mm 1*square_mm 0*square_mm];
worldPoints_cal(3,1:3)=[3*square_mm 1*square_mm 0*square_mm];
worldPoints_cal(4,1:3)=[4*square_mm 1*square_mm 0*square_mm];
worldPoints_cal(5,1:3)=[5*square_mm 1*square_mm 0*square_mm];
worldPoints_cal(6,1:3)=[1*square_mm 2*square_mm 0*square_mm];
worldPoints_cal(7,1:3)=[2*square_mm 2*square_mm 0*square_mm];
worldPoints_cal(8,1:3)=[3*square_mm 2*square_mm 0*square_mm];
worldPoints_cal(9,1:3)=[4*square_mm 2*square_mm 0*square_mm];
worldPoints_cal(10,1:3)=[5*square_mm 2*square_mm 0*square_mm];
worldPoints_cal(11,1:3)=[1*square_mm 3*square_mm 0*square_mm];
worldPoints_cal(12,1:3)=[2*square_mm 3*square_mm 0*square_mm];
worldPoints_cal(13,1:3)=[3*square_mm 3*square_mm 0*square_mm];
worldPoints_cal(14,1:3)=[4*square_mm 3*square_mm 0*square_mm];
worldPoints_cal(15,1:3)=[5*square_mm 3*square_mm 0*square_mm];
worldPoints_cal(16,1:3)=[1*square_mm 4*square_mm 0*square_mm];
worldPoints_cal(17,1:3)=[2*square_mm 4*square_mm 0*square_mm];
worldPoints_cal(18,1:3)=[3*square_mm 4*square_mm 0*square_mm];
worldPoints_cal(19,1:3)=[4*square_mm 4*square_mm 0*square_mm];
worldPoints_cal(20,1:3)=[5*square_mm 4*square_mm 0*square_mm];
end
%%
worldPoints_cal=worldPoints_cal';
imagePoints_cal=imagePoints_cal';
%% Plotting points use for HM 
W=repmat([250],1,1);
w=W(:);
figure(8)
hold on
grid on
axis equal
for i=1:size(imagePoints_cal,2)
    scatter3(imagePoints_cal(1,i),imagePoints_cal(2,i),imagePoints_cal(3,i),w,'r*')
    hold on
    pause(0.01)
    figure(8)
    hold on
    scatter3(worldPoints_cal(1,i),worldPoints_cal(2,i),worldPoints_cal(3,i),w,'k*')
    pause(0.01)
end

%% Interopolation of a plane - in case of planar aquisition (the choosen points belong all to a plane)
% Definition of the mean error in term of distance of point from the plane 
for d=1

imagePoints_cal=imagePoints_cal';
[no,V,p]=affine_fit(imagePoints_cal);

% How to generate and plot the plane

d=-p*no;
% [xx,yy]=ndgrid(0.02:0.10,0.02:0.10);
% [xx,yy]=ndgrid(-0.02:-0.01:-0.12,-0.02:-0.01:-0.10);
[xx,yy]=ndgrid(-50:1:20,0:1:60);

figure(8)
hold on
z=(-no(1)*xx-no(2)*yy-d)/no(3);
su=surf(xx,yy,z);
su.EdgeColor='none';
su.FaceColor=[1 0 0];
su.FaceAlpha=0.3;

%%  calculating the distance of each point from the plane
a=no(1);
b=no(2);
c=no(3);
imagePoints_cal=imagePoints_cal';
for i=1:size(imagePoints_cal,2)

    v(:,i) = point_plane_shortest_dist_vec(imagePoints_cal(1,i),imagePoints_cal(2,i),imagePoints_cal(3,i), a, b, c, d);
    dist(:,i)=norm(v(:,i));
end


err_dist=mean(dist)
err_max=max(dist)
std_err=std(dist)
end

%% Matching the two cloud of point through HORN METHODS
[Text_horn_mm] = quaternion_matching(worldPoints_cal,imagePoints_cal);
[regParams,Bfit,ErrorStats]=absor(worldPoints_cal,imagePoints_cal); % 3xN
M=regParams.M;
%% Applying the transformation to the points 
RF_ws=createRF([1*square_mm;0;0],[4*square_mm;0;0],[0;2*square_mm;0;],[0;0;0]);
% RF_im=Text_horn_mm*RF_ws;
RF_im=M*RF_ws;
% RF_im=Text_icp_mm*RF_ws;
figure(8)
hold on
trplot(RF_ws,'length',20,'color','k');
hold on
trplot(RF_im,'length',20,'color','r');
axis equal

%% Applying the transformation to the point - CHEKING PHASE
worldPoints_cal_h=worldPoints_cal;
worldPoints_cal_h(4,:)=ones();
 
R=repmat([150],1,1);
r=R(:);
for i=1:size(worldPoints_cal,2)
    worldPoints_cal_ck(:,i)=M*worldPoints_cal_h(:,i);
    figure(8)
    hold on
    scatter3(worldPoints_cal_ck(1,i),worldPoints_cal_ck(2,i),worldPoints_cal_ck(3,i),r,'r','filled')
    hold on
end

worldPoints_cal_ck=worldPoints_cal_ck(1:3,:);

%% Calculating the distance     -   ACCURACY DEFINITION OF THE TRANSFORMATION 
imagePoints_cal=imagePoints_cal';
%%
for i=1:size(worldPoints_cal_ck,2)
    
    acc_ext_HM(:,i)=abs(imagePoints_cal(:,i)-worldPoints_cal_ck(:,i));
    dist_ext_HM(i)=sqrt(acc_ext_HM(1,i)^2+acc_ext_HM(2,i)^2);
    
end

mean_err_ext_HM=mean(dist_ext_HM);
dev_ext_HM=std(dist_ext_HM);




