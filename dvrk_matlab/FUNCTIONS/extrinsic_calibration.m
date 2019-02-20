% This function compute the extrinsic calibration based on the intrisic one
% INPUT PARAMETERS:
%                   - im: image after being read (im=imread('Image_L1.jpg'))
%                   - InstrinsicParam: matrix with the intrinsic parameters
%                   related to the camera
%                   - squareSize: size of the chess board express in [mm].
% OUTPUT PARAMETERS:
%                   - rotationMatrix: represent the rotational matrix
%                   - translationVector
%                   - Text: represent the T matrix of the pose of the
%                   camera

function [rotationMatrix,translationVector,Text]=extrinsic_calibration(im,intrinsicParam,squareSize)

[imm] = undistortImage(im,intrinsicParam);

for i=1:5
    
    figure(10)
    imshow(imm)
    % [x,y,value]=impixel(imm)
    squaredim=[5 4];
    points=squaredim(1,1)*squaredim(1,2);
    [x,y]=ginput(points);
    close
    imagePoints(:,1,i)=x;
    imagePoints(:,2,i)=y;
end 

imagePoints_mean(:,:)=mean(imagePoints,3);
boardSize = [ squaredim(1,1)+1 , squaredim(1,2)+1 ] ;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
[rotationMatrix,translationVector] = extrinsics(imagePoints_mean,worldPoints,intrinsicParam);

Text=zeros(4);
Text(1:3,1:3)=rotationMatrix;
Text(1:3,4)=translationVector;
Text(4,4)=1;


end


