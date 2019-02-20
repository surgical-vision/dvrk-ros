%% COMPARISON OF THE RESULTS

load('cameraParams_L.mat')
load('cc_tip_mean_L');
load('rotationMatrix.mat');
load('translationVector.mat');

%% Definition of the extrinsic trasnformation matrix

Text=zeros(4);
Text(1:3,1:3)=rotationMatrix;
Text(1:3,4)=translationVector;
Text(4,4)=1;

% %% Matching of the chessboard point 
% cc_point=zeros(4,20);
% cc_point(1:2,:)=imagePoints';
% cc_point(4,:)=ones(1,20);
% 
% % undistortedPoints = undistortPoints(imagePoints,intrinsic);
% worldPoints = pointsToWorld(cameraParams,rotationMatrix,translationVector,imagePoints);
% wpoint=zeros(3,20);
% wpoint(1:2,:)=worldPoints';
% wpoint=wpoint./1000;
% %% Plotting 
% figure(1)
% hold on
% scatter3(point(1,:),wpoint(2,:),wpoint(3,:),'k*')
% grid on;

%% Definition of the point in the ws

tipPoints = pointsToWorld(cameraParams,rotationMatrix,translationVector,cc_tip_mean_L');

tpoint=zeros(3,11);

tpoint(1:2,:)=tipPoints';
tpoint=tpoint./1000;
tpoint=tpoint(:,2:11);
%% Plotting of the points 
figure(1)
hold on
scatter3(tpoint(1,:),tpoint(2,:),tpoint(3,:),'g*')
grid on;

%% Quantification of the accuracy

ppoint=point(1:3,:);

for i=1:size(point,2)
    
    acc(:,i)=abs(ppoint(:,i)-tpoint(:,i));
    dist(i)=sqrt(acc(1,i)^2+acc(2,i)^2);
    
end

mean_err=mean(dist);

%% Try to determine the shifth in the tip point

[regParams,Bfit,ErrorStats]=absor(point(1:3,:),tpoint);
T_shift=regParams.M;

tpoint_om=ones(4,length(tpoint));
tpoint_om(1:3,:)=tpoint;

point_shift=inv(T_shift)*tpoint_om;

figure(1)
hold on
scatter3(point_shift(1,:),point_shift(2,:),point_shift(3,:),'m*')
grid on;


