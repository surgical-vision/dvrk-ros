%% COMPARISON OF THE RESULTS

load('cameraParams_L.mat')
% load('cc_tip_mean_L');
load('rotationMatrix.mat');
load('translationVector.mat');

%% Definition of the extrinsic transformation matrix

Text=zeros(4);
Text(1:3,1:3)=rotationMatrix;
Text(1:3,4)=translationVector;
Text(4,4)=1;
% Adding the rigid translation
Text(1,4)=Text(1,4)+10;
Text(2,4)=Text(2,4)+10;