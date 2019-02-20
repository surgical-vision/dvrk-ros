% This function compute the extrinsic calibration based on the intrisic one
function [rotationMatrix,translationVector,Text]=extrinsic_calibration(im,instrinsicParam)

[imm] = undistortImage(im,cameraParams_L,'OutputView','full');
figure(1)
imshow(imm)
imagePoints=impixel
close figure(1)






end


