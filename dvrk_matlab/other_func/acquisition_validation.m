%% VERIFY THAT ALL THE POINTS ACQUIRED BELONG TO A PLANE
close all
clear all
clc

%% Loading the point for each planes
load ccXY;
load cjointXY;
ccXY=cc;
cjointXY=cjoint;
teleopXY=ccXY;

load ccXZ;
load cjointXZ;
ccXZ=cc;
cjointXZ=cjoint;
teleopXZ=ccXZ;

load ccYZ;
load cjointYZ;
ccYZ=cc;
cjointYZ=cjoint;
teleopYZ=ccYZ;

%% Plotting of the points acquired for each plane
np=size(teleopXY,3);

for i=1:np
    teleXY(:,i)=teleopXY(1:3,4,i); % create 2D matrix with cartesian coordinate
    teleXZ(:,i)=teleopXZ(1:3,4,i); 
    teleYZ(:,i)=teleopYZ(1:3,4,i); 

end

W=repmat([350],1,1);
w=W(:);
figure(1)
hold on
grid on
for i=1:np
    scatter3(teleXY(1,i),teleXY(2,i),teleXY(3,i),w,'b*');
    hold on
    scatter3(teleXZ(1,i),teleXZ(2,i),teleXZ(3,i),w,'r*');
    hold on
    scatter3(teleYZ(1,i),teleYZ(2,i),teleYZ(3,i),w,'k*');
end
axis equal
 %% Try to fit the planes
 
% XY
teleXYp=teleXY';
[nXY,VXY,pXY]=affine_fit(teleXYp);

% How to generate and plot the plane
nXY=nXY';

dXY=-pXY*nXY';
% [xx,yy]=ndgrid(0.02:0.10,0.02:0.10);
% [xx,yy]=ndgrid(-0.02:-0.01:-0.12,-0.02:-0.01:-0.10);
[xxXY,yyXY]=ndgrid(0.05:-0.01:-0.06,0.03:-0.01:-0.10);

figure(1)
hold on
zXY=(-nXY(1)*xxXY-nXY(2)*yyXY-dXY)/nXY(3);
suXY=surf(xxXY,yyXY,zXY)
suXY.EdgeColor='none';
suXY.FaceColor=[0 0 1];
suXY.FaceAlpha=0.3;

%% This plane behave really weired 
% % XZ
% teleXZp=teleXZ';
% [nXZ,VXZ,pXZ]=affine_fit(teleXZp);
% 
% % How to generate and plot the plane
% nXZ=nXZ';
% 
% dXZ=-pXZ*nXZ';
% % [xx,yy]=ndgrid(0.02:0.10,0.02:0.10);
% % [xx,yy]=ndgrid(-0.02:-0.01:-0.12,-0.02:-0.01:-0.10);
% [xxXZ,yyXZ]=ndgrid(0.06:-0.01:-0.06,0.06:-0.01:-0.06);
% 
% 
% figure(1)
% hold on
% zXZ=(-nXZ(1)*xxXZ-nXZ(2)*yyXZ-dXZ)/nXZ(3);
% suXZ=surf(xxXZ,yyXZ,zXZ)
% suXZ.EdgeColor='none';
% suXZ.FaceColor=[1 0 0];
% suXZ.FaceAlpha=0.3;

%% YZ
teleYZp=teleYZ';
[nYZ,VYZ,pYZ]=affine_fit(teleYZp);

% How to generate and plot the plane
nYZ=nYZ';

dYZ=-pYZ*nYZ';
% [xx,yy]=ndgrid(0.02:0.10,0.02:0.10);
% [xx,yy]=ndgrid(-0.02:-0.01:-0.12,-0.02:-0.01:-0.10);
% [xxYZ,yyYZ]=ndgrid(-0.01:0.01:0.10,-0.01:0.01:0.10);
[xxYZ,yyYZ]=ndgrid(0.06:-0.01:-0.07,0.03:-0.01:-0.10);


figure(1)
hold on
zYZ=(-nYZ(1)*xxYZ-nYZ(2)*yyYZ-dYZ)/nYZ(3);
suYZ=surf(xxYZ,yyYZ,zYZ)
suYZ.EdgeColor='none';
suYZ.FaceColor=[0 0 0];
suYZ.FaceAlpha=0.2;



