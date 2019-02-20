%% change path
cameraParams_ = load('cameraParams_L.mat');
cameraParams = cameraParams_.cameraParams;
im = imread('Image_L1.jpg');

[im] = undistortImage(im,cameraParams,'OutputView','full');

[imagePoints_,boardSize_] = detectCheckerboardPoints(im);
imagePoints = imagePoints_(5:end,:);


       

figure();
imshow(im); hold 'on';
for idx = 1:size(imagePoints,1)
   plot(imagePoints(idx,1),imagePoints(idx,2),'go'); hold 'on';
   pause(1)
end

%% change square size if necessary
squareSize = 10;
boardSize = [ boardSize_(1) , boardSize_(2)-1 ] ;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%%
[rotationMatrix,translationVector] = extrinsics(imagePoints,worldPoints,cameraParams);
rotationMatrix
translationVector
%%
[orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
  translationVector);
figure
plotCamera('Location',location,'Orientation',orientation,'Size',4);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
  'VerticalAxisDir','down','MarkerSize',40);


figure()
imshow(im); hold 'on';
imgPts = worldToImage(cameraParams, rotationMatrix, translationVector, [15,0,0]);
plot( imgPts(1),imgPts(2),'g*');

