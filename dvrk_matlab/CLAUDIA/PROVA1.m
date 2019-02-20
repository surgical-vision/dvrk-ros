% generation of the undistort images 
load('stereoParams_new.mat');
camera1= stereoParams_new.CameraParameters1;
camera2= stereoParams_new.CameraParameters2;

L_cam= imread('LEFT-P.png');
R_cam= imread('RIGHT-P.png');

% detect 2D checkerboard points
[ L_imagePoints , L_boardSize ] = detectCheckerboardPoints(L_cam) ;
[ R_imagePoints , R_boardSize ] = detectCheckerboardPoints(R_cam) ;

% undistort the point
L_imagePoints_UN= undistortPoints(L_imagePoints,camera1);
R_imagePoints_UN= undistortPoints(R_imagePoints,camera2);

worldPoints_check_un=triangulate(L_imagePoints_UN,R_imagePoints_UN,stereoParams_new);

figure(2)
hold on
for i=1:size(worldPoints_check_un,1)
%     plot(L_imagePoints(i,1),L_imagePoints(i,2),'r*');
    hold on
    grid on
%     plot(R_imagePoints(i,1),R_imagePoints(i,2),'b*');
%     hold on
    scatter3(worldPoints_check_un(i,1),worldPoints_check_un(i,2),worldPoints_check_un(i,3),'b*');
    pause(0.1)
end