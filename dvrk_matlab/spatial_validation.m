%% ANALYSIS OF THE THREE PLANES
close all
clear all
clc
%% Better matrix formulation (loading of the point saved)
load ccxy;
load cjointxy;
ccxy=cc;
cjointxy=cjoint;

load ccxz;
load cjointxz;
ccxz=cc;
cjointxz=cjoint;

load ccyz;
load cjointyz;
ccyz=cc;
cjointyz=cjoint;

%% Better formulation of the points - for calibration for SINGLE PLANE
teleopxy=ccxy;
teleopxz=ccxz;
teleopyz=ccyz;
np=size(teleopxy,3);

for i=1:np
    telexy(:,i)=teleopxy(1:3,4,i); % create 2D matrix with cartesian coordinate
    telexz(:,i)=teleopxz(1:3,4,i);
    teleyz(:,i)=teleopyz(1:3,4,i);
end

%% Creation on xy-plane
square= 0.010;

wsxy(1:3,1)=[2*square;2*square;0*square];
wsxy(1:3,2)=[4*square;2*square;0*square];
wsxy(1:3,3)=[6*square;2*square;0*square];
wsxy(1:3,4)=[6*square;4*square;0*square];
wsxy(1:3,5)=[6*square;6*square;0*square];
wsxy(1:3,6)=[4*square;6*square;0*square];
wsxy(1:3,7)=[2*square;6*square;0*square];
wsxy(1:3,8)=[2*square;4*square;0*square];

%% Creation on xz-plane
square= 0.010;

wsxz(1:3,1)=[2*square;0*square;2*square];
wsxz(1:3,2)=[4*square;0*square;2*square];
wsxz(1:3,3)=[6*square;0*square;2*square];
wsxz(1:3,4)=[6*square;0*square;4*square];
wsxz(1:3,5)=[6*square;0*square;6*square];
wsxz(1:3,6)=[4*square;0*square;6*square];
wsxz(1:3,7)=[2*square;0*square;6*square];
wsxz(1:3,8)=[2*square;0*square;4*square];


%% Creation on yz-plane
square= 0.010;

wsyz(1:3,1)=[0*square;2*square;2*square];
wsyz(1:3,2)=[0*square;4*square;2*square];
wsyz(1:3,3)=[0*square;6*square;2*square];
wsyz(1:3,4)=[0*square;6*square;4*square];
wsyz(1:3,5)=[0*square;6*square;6*square];
wsyz(1:3,6)=[0*square;4*square;6*square];
wsyz(1:3,7)=[0*square;2*square;6*square];
wsyz(1:3,8)=[0*square;2*square;4*square];

%% Visualization of the two points clouds
W=repmat([350],1,1);
w=W(:);
%%
figure(1)
hold on
grid on
for i=1:np
    scatter3(telexy(1,i),telexy(2,i),telexy(3,i),w,'b*');
    hold on
    scatter3(wsxy(1,i),wsxy(2,i),wsxy(3,i),w,'r*');
    pause(0.1)
end
hold on 
title('Distribution of the aquired points XY-PLANE')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')
axis equal

%%
figure(2)
hold on
grid on
for i=1:np
    scatter3(telexz(1,i),telexz(2,i),telexz(3,i),w,'b*');
    hold on
    scatter3(wsxz(1,i),wsxz(2,i),wsxz(3,i),w,'r*');
    pause(0.1)
end
hold on 
title('Distribution of the aquired points XZ-PLANE')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')
axis equal
%%
figure(3)
hold on
grid on
for i=1:np
    scatter3(teleyz(1,i),teleyz(2,i),teleyz(3,i),w,'b*');
    hold on
    scatter3(wsyz(1,i),wsyz(2,i),wsyz(3,i),w,'r*');
    pause(0.1)
end
hold on 
title('Distribution of the aquired points YZ-PLANE')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')
axis equal

%% Create RF

RF_ws=createRF([1*square;0;0],[4*square;0;0],[0;2*square;0;],[0;0;0]);

figure(1)
hold on
trplot(RF_ws,'length',0.02,'color','r');
figure(2)
hold on
trplot(RF_ws,'length',0.02,'color','r');
figure(3)
hold on
trplot(RF_ws,'length',0.02,'color','r');

%% Mapping the two clouds of points through Horn's methods.
% creation of the transformation matrix

[Txy] = quaternion_matching(wsxy,telexy);   % transformation from ws->rc
[Txz] = quaternion_matching(wsxz,telexz);
[Tyz] = quaternion_matching(wsyz,teleyz);

RF_rcxy=Txy*RF_ws;
RF_rcxz=Txz*RF_ws;
RF_rcyz=Tyz*RF_ws;

%% Plotting RF_PSM
figure(1)
hold on
trplot(RF_rcxy,'length',0.02,'color','b')
figure(2)
hold on
trplot(RF_rcxz,'length',0.02,'color','b')
figure(3)
hold on
trplot(RF_rcyz,'length',0.02,'color','b')
%% Checking of the transformation 
% Plotting of the points 
wsxy_h=wsxy;
wsxy_h(4,:)=ones();
wsxz_h=wsxz;
wsxz_h(4,:)=ones();
wsyz_h=wsyz;
wsyz_h(4,:)=ones();
 
R=repmat([150],1,1);
r=R(:);
for i=1:np
    point_psm_ckxy(:,i)=Txy*wsxy_h(:,i);
    figure(1)
    hold on
    scatter3(point_psm_ckxy(1,i),point_psm_ckxy(2,i),point_psm_ckxy(3,i),r,'b','filled')
    hold on
end
for i=1:np
    point_psm_ckxz(:,i)=Txz*wsxz_h(:,i);
    figure(2)
    hold on
    scatter3(point_psm_ckxz(1,i),point_psm_ckxz(2,i),point_psm_ckxz(3,i),r,'b','filled')
    hold on
end
for i=1:np
    point_psm_ckyz(:,i)=Tyz*wsyz_h(:,i);
    figure(3)
    hold on
    scatter3(point_psm_ckyz(1,i),point_psm_ckyz(2,i),point_psm_ckyz(3,i),r,'b','filled')
    hold on
end









%% VALIDATION OF TWO PLANES AT THE SAME TIME 

%% for XY-XZ space
for i=1:np
    telexyxz(:,i)=telexy(:,i); % create 2D matrix with cartesian coordinate
    telexyxz(:,i+8)=telexz(:,i);
end
%% for XZ-YZ space
for i=1:np
    telexzyz(:,i)=telexz(:,i); % create 2D matrix with cartesian coordinate
    telexzyz(:,i+8)=teleyz(:,i);
end
%% for XY-YZ space
for i=1:np
    telexyyz(:,i)=telexy(:,i); % create 2D matrix with cartesian coordinate
    telexyyz(:,i+8)=teleyz(:,i);
end
%% Creation on xy-plane and xz-plane
square= 0.010;

wsxyxz(1:3,1)=[2*square;2*square;0*square];
wsxyxz(1:3,2)=[4*square;2*square;0*square];
wsxyxz(1:3,3)=[6*square;2*square;0*square];
wsxyxz(1:3,4)=[6*square;4*square;0*square];
wsxyxz(1:3,5)=[6*square;6*square;0*square];
wsxyxz(1:3,6)=[4*square;6*square;0*square];
wsxyxz(1:3,7)=[2*square;6*square;0*square];
wsxyxz(1:3,8)=[2*square;4*square;0*square];
wsxyxz(1:3,9)=[2*square;0*square;2*square];
wsxyxz(1:3,10)=[4*square;0*square;2*square];
wsxyxz(1:3,11)=[6*square;0*square;2*square];
wsxyxz(1:3,12)=[6*square;0*square;4*square];
wsxyxz(1:3,13)=[6*square;0*square;6*square];
wsxyxz(1:3,14)=[4*square;0*square;6*square];
wsxyxz(1:3,15)=[2*square;0*square;6*square];
wsxyxz(1:3,16)=[2*square;0*square;4*square];

%% Creation on xz-plane and yz-plane
square= 0.010;

wsxzyz(1:3,1)=[2*square;0*square;2*square];
wsxzyz(1:3,2)=[4*square;0*square;2*square];
wsxzyz(1:3,3)=[6*square;0*square;2*square];
wsxzyz(1:3,4)=[6*square;0*square;4*square];
wsxzyz(1:3,5)=[6*square;0*square;6*square];
wsxzyz(1:3,6)=[4*square;0*square;6*square];
wsxzyz(1:3,7)=[2*square;0*square;6*square];
wsxzyz(1:3,8)=[2*square;0*square;4*square];
wsxzyz(1:3,9)=[0*square;2*square;2*square];
wsxzyz(1:3,10)=[0*square;4*square;2*square];
wsxzyz(1:3,11)=[0*square;6*square;2*square];
wsxzyz(1:3,12)=[0*square;6*square;4*square];
wsxzyz(1:3,13)=[0*square;6*square;6*square];
wsxzyz(1:3,14)=[0*square;4*square;6*square];
wsxzyz(1:3,15)=[0*square;2*square;6*square];
wsxzyz(1:3,16)=[0*square;2*square;4*square];

%% Creation on xy-plane and yz-plane
square= 0.010;

wsxyyz(1:3,1)=[2*square;2*square;0*square];
wsxyyz(1:3,2)=[4*square;2*square;0*square];
wsxyyz(1:3,3)=[6*square;2*square;0*square];
wsxyyz(1:3,4)=[6*square;4*square;0*square];
wsxyyz(1:3,5)=[6*square;6*square;0*square];
wsxyyz(1:3,6)=[4*square;6*square;0*square];
wsxyyz(1:3,7)=[2*square;6*square;0*square];
wsxyyz(1:3,8)=[2*square;4*square;0*square];
wsxyyz(1:3,9)=[0*square;2*square;2*square];
wsxyyz(1:3,10)=[0*square;4*square;2*square];
wsxyyz(1:3,11)=[0*square;6*square;2*square];
wsxyyz(1:3,12)=[0*square;6*square;4*square];
wsxyyz(1:3,13)=[0*square;6*square;6*square];
wsxyyz(1:3,14)=[0*square;4*square;6*square];
wsxyyz(1:3,15)=[0*square;2*square;6*square];
wsxyyz(1:3,16)=[0*square;2*square;4*square];

%% Visualization of the two points clouds
W=repmat([350],1,1);
w=W(:);
%%
figure(4)
hold on
grid on
for i=1:(2*np)
    scatter3(telexyxz(1,i),telexyxz(2,i),telexyxz(3,i),w,'b*');
    hold on
    scatter3(wsxyxz(1,i),wsxyxz(2,i),wsxyxz(3,i),w,'r*');
    axis equal
    pause(0.1)
end
hold on 
title('Distribution of the aquired points XY/XZ-PLANE')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')
% axis equal

%%
figure(5)
hold on
grid on
for i=1:(2*np)
    scatter3(telexzyz(1,i),telexzyz(2,i),telexzyz(3,i),w,'b*');
    hold on
    scatter3(wsxzyz(1,i),wsxzyz(2,i),wsxzyz(3,i),w,'r*');
    pause(0.1)
end
hold on 
title('Distribution of the aquired points XZ/YZ-PLANE')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')
axis equal

%%
figure(6)
hold on
grid on
for i=1:(2*np)
    scatter3(telexyyz(1,i),telexyyz(2,i),telexyyz(3,i),w,'b*');
    hold on
    scatter3(wsxyyz(1,i),wsxyyz(2,i),wsxyyz(3,i),w,'r*');
    pause(0.1)
end
hold on 
title('Distribution of the aquired points XY/YZ-PLANE')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')
axis equal

%% Create RF

RF_ws=createRF([1*square;0;0],[4*square;0;0],[0;2*square;0;],[0;0;0]);

figure(4)
hold on
trplot(RF_ws,'length',0.02,'color','r');
figure(5)
hold on
trplot(RF_ws,'length',0.02,'color','r');
figure(6)
hold on
trplot(RF_ws,'length',0.02,'color','r');

%% Mapping the two clouds of points through Horn's methods.
% creation of the transformation matrix

[Txyxz] = quaternion_matching(wsxyxz,telexyxz);   % transformation from ws->rc
[Txzyz] = quaternion_matching(wsxzyz,telexzyz);
[Txyyz] = quaternion_matching(wsxyyz,telexyyz);

RF_rcxyxz=Txyxz*RF_ws;
RF_rcxzyz=Txzyz*RF_ws;
RF_rcxyyz=Txyyz*RF_ws;

%% Plotting RF_PSM
figure(4)
hold on
trplot(RF_rcxyxz,'length',0.02,'color','b')
figure(5)
hold on
trplot(RF_rcxzyz,'length',0.02,'color','b')
figure(6)
hold on
trplot(RF_rcxyyz,'length',0.02,'color','b')

%% Checking of the transformation 
% Plotting of the points 
wsxyxz_h=wsxyxz;
wsxyxz_h(4,:)=ones();
wsxzyz_h=wsxzyz;
wsxzyz_h(4,:)=ones();
wsxyyz_h=wsxyyz;
wsxyyz_h(4,:)=ones();
 
R=repmat([150],1,1);
r=R(:);
for i=1:(2*np)
    point_psm_ckxyxz(:,i)=Txyxz*wsxyxz_h(:,i);
    figure(4)
    hold on
    scatter3(point_psm_ckxyxz(1,i),point_psm_ckxyxz(2,i),point_psm_ckxyxz(3,i),r,'b','filled')
    hold on
end
for i=1:(2*np)
    point_psm_ckxzyz(:,i)=Txzyz*wsxzyz_h(:,i);
    figure(5)
    hold on
    scatter3(point_psm_ckxzyz(1,i),point_psm_ckxzyz(2,i),point_psm_ckxzyz(3,i),r,'b','filled')
    hold on
end
for i=1:(2*np)
    point_psm_ckxyyz(:,i)=Txyyz*wsxyyz_h(:,i);
    figure(6)
    hold on
    scatter3(point_psm_ckxyyz(1,i),point_psm_ckxyyz(2,i),point_psm_ckxyyz(3,i),r,'b','filled')
    hold on
end



%% PUTTING TOGETHER THE THREE PLANES
%% for XY-XZ-YZ space
for i=1:np
    telexyxzyz(:,i)=telexy(:,i); % create 2D matrix with cartesian coordinate
    telexyxzyz(:,i+8)=telexz(:,i);
    telexyxzyz(:,i+16)=teleyz(:,i);
end

%% Creation on xy-plane and xz-plane
square= 0.010;

wsxyxzyz(1:3,1)=[2*square;2*square;0*square];
wsxyxzyz(1:3,2)=[4*square;2*square;0*square];
wsxyxzyz(1:3,3)=[6*square;2*square;0*square];
wsxyxzyz(1:3,4)=[6*square;4*square;0*square];
wsxyxzyz(1:3,5)=[6*square;6*square;0*square];
wsxyxzyz(1:3,6)=[4*square;6*square;0*square];
wsxyxzyz(1:3,7)=[2*square;6*square;0*square];
wsxyxzyz(1:3,8)=[2*square;4*square;0*square];
wsxyxzyz(1:3,9)=[2*square;0*square;2*square];
wsxyxzyz(1:3,10)=[4*square;0*square;2*square];
wsxyxzyz(1:3,11)=[6*square;0*square;2*square];
wsxyxzyz(1:3,12)=[6*square;0*square;4*square];
wsxyxzyz(1:3,13)=[6*square;0*square;6*square];
wsxyxzyz(1:3,14)=[4*square;0*square;6*square];
wsxyxzyz(1:3,15)=[2*square;0*square;6*square];
wsxyxzyz(1:3,16)=[2*square;0*square;4*square];
wsxyxzyz(1:3,17)=[0*square;2*square;2*square];
wsxyxzyz(1:3,18)=[0*square;4*square;2*square];
wsxyxzyz(1:3,19)=[0*square;6*square;2*square];
wsxyxzyz(1:3,20)=[0*square;6*square;4*square];
wsxyxzyz(1:3,21)=[0*square;6*square;6*square];
wsxyxzyz(1:3,22)=[0*square;4*square;6*square];
wsxyxzyz(1:3,23)=[0*square;2*square;6*square];
wsxyxzyz(1:3,24)=[0*square;2*square;4*square];


%% Plotting the points
figure(7)
hold on
grid on
for i=1:(3*np)
    scatter3(telexyxzyz(1,i),telexyxzyz(2,i),telexyxzyz(3,i),w,'b*');
    hold on
    scatter3(wsxyxzyz(1,i),wsxyxzyz(2,i),wsxyxzyz(3,i),w,'r*');
%     axis([-0.5 0.5 -0.1 0.1 -0.5 0.5])
%     axis equal
    pause(0.1)
end
hold on 
title('Distribution of the aquired points XY/XZ/YZ-PLANE')
xlabel('[m]')
ylabel('[m]')
zlabel('[m]')
% axis equal

%%
RF_ws=createRF([1*square;0;0],[4*square;0;0],[0;2*square;0;],[0;0;0]);

figure(7)
hold on
trplot(RF_ws,'length',0.02,'color','r');

%% Mapping the two clouds of points through Horn's methods.
% creation of the transformation matrix

[Txyxzyz] = quaternion_matching(wsxyxzyz,telexyxzyz);
RF_rcxyxzyz=Txyxzyz*RF_ws;

figure(7)
hold on
trplot(RF_rcxyxzyz,'length',0.02,'color','b')
%% Checking of the transformation 
% Plotting of the points 
wsxyxzyz_h=wsxyxzyz;
wsxyxzyz_h(4,:)=ones();

 
R=repmat([150],1,1);
r=R(:);
for i=1:(3*np)
    point_psm_ckxyxzyz(:,i)=Txyxzyz*wsxyxzyz_h(:,i);
    figure(7)
    hold on
    scatter3(point_psm_ckxyxzyz(1,i),point_psm_ckxyxzyz(2,i),point_psm_ckxyxzyz(3,i),r,'b','filled')
    hold on
end





%% Motion of the PSM
r = arm('PSM1');

%%
point(1:3,1)=[0*square; 0*square; 5*square];
point(1:3,2)=[1*square; 1*square; 5*square];
point(1:3,3)=[1*square; 1*square; 4*square];
point(1:3,4)=[2*square; 2*square; 4*square];
point(1:3,5)=[2*square; 2*square; 3*square];
point(1:3,6)=[3*square; 3*square; 3*square];
point(1:3,7)=[3*square; 3*square; 2*square];
point(1:3,8)=[4*square; 4*square; 2*square];
point(1:3,9)=[4*square; 4*square; 1*square];
point(1:3,10)=[5*square; 5*square; 1*square];
point(1:3,11)=[5*square; 5*square; 0*square];
point(4,:)=ones();

for i=1:11
    point_psm(:,i)=Txyxzyz*point(:,i);
end

%% Motion along the traj of points
pointc=point_psm(1:3,:);

for i=1:11
    move_translation(r,pointc(:,i)')
    pause(0.5)
end





%% SCANNING OF THE PLANES

r = arm('PSM1');
%% XY - plane
max=400;
for i=1:max
    c1=r.position_joint_current;
    cjoint(:,:,i)=c1;  
    c1c=r.position_current(:,:);
    cc(:,:,i)=c1c;  
    pause(0.3)
end 
%%
load('ccXY');
load('cjointXY');
%%
teleopXY=cc;
np=size(cc,3);

for i=1:np
    teleXY(:,i)=teleopXY(1:3,4,i); 
end


%%
W=repmat([50],1,1);
w=W(:);
figure(1)
hold on
grid on
for i=1:np
    hold on
    grid on
    scatter3(teleXY(1,i),teleXY(2,i),teleXY(3,i),w,'b');
end
axis equal


%%
telepXY=teleXY';
[noXY,VXY,pXY]=affine_fit(telepXY);

% How to generate and plot the plane

dXY=-pXY*noXY;
% [xx,yy]=ndgrid(0.02:0.10,0.02:0.10);
% [xx,yy]=ndgrid(-0.02:-0.01:-0.12,-0.02:-0.01:-0.10);
[xxXY,yyXY]=ndgrid(0.05:-0.01:-0.06,0.03:-0.01:-0.10);

figure(1)
hold on
zXY=(-noXY(1)*xxXY-noXY(2)*yyXY-dXY)/noXY(3);
suXY=surf(xxXY,yyXY,zXY);
suXY.EdgeColor='none';
suXY.FaceColor=[0 0 1];
suXY.FaceAlpha=0.3;

%%  calculating the distance of each point from the plane
aXY=noXY(1);
bXY=noXY(2);
cXY=noXY(3);
max=np;
for i=1:max

    vXY(:,i) = point_plane_shortest_dist_vec(teleXY(1,i),teleXY(2,i),teleXY(3,i), aXY, bXY, cXY, dXY);
    distXY(:,i)=norm(vXY(:,i));
end


err_distXY=mean(distXY);
std_errXY=std(distXY);


%% XZ - plane
max=400;
for i=1:max
    c1=r.position_joint_current;
    cjoint(:,:,i)=c1;  
    c1c=r.position_current(:,:);
    cc(:,:,i)=c1c;  
    pause(0.3)
end 
%%
load('ccXZ');
load('cjointXZ');
%%
teleopXZ=cc;
np=size(cc,3);

for i=1:np
    teleXZ(:,i)=teleopXZ(1:3,4,i); 
end


%%
W=repmat([50],1,1);
w=W(:);
figure(2)
hold on
grid on
for i=1:np
    hold on
    grid on
    scatter3(teleXZ(1,i),teleXZ(2,i),teleXZ(3,i),w,'r');
end
axis equal


%%
telepXZ=teleXZ';
[noXZ,VXZ,pXZ]=affine_fit(telepXZ);

% How to generate and plot the plane

dXZ=-pXZ*noXZ;
% [xxXZ,yyXZ]=ndgrid(0.02:0.01:0.10,0.02:0.01:0.10);
% [xx,yy]=ndgrid(-0.02:-0.01:-0.12,-0.02:-0.01:-0.10);
[xxXZ,yyXZ]=ndgrid(0.05:-0.01:-0.06,0.03:-0.01:-0.10);

figure(2)
hold on
zXZ=(-noXZ(1)*xxXZ-noXZ(2)*yyXZ-dXZ)/noXZ(3);
suXZ=surf(xxXZ,yyXZ,zXZ);
suXZ.EdgeColor='none';
suXZ.FaceColor=[1 0 0];
suXZ.FaceAlpha=0.3;

%%  calculating the distance of each point from the plane
aXZ=noXZ(1);
bXZ=noXZ(2);
cXZ=noXZ(3);
max=np;
for i=1:max

    vXZ(:,i)= point_plane_shortest_dist_vec(teleXZ(1,i),teleXZ(2,i),teleXZ(3,i), aXZ, bXZ, cXZ, dXZ);
    distXZ(:,i)=norm(vXZ(:,i));
end


err_distXZ=mean(distXZ);
std_errXZ=std(distXZ);




%% YZ - plane
max=400;
for i=1:max
    c1=r.position_joint_current;
    cjoint(:,:,i)=c1;  
    c1c=r.position_current(:,:);
    cc(:,:,i)=c1c;  
    pause(0.3)
end 
%%
load('ccYZ');
load('cjointYZ');
%%
teleopYZ=cc;
np=size(cc,3);

for i=1:np
    teleYZ(:,i)=teleopYZ(1:3,4,i); 
end


%%
W=repmat([50],1,1);
w=W(:);
figure(2)
hold on
grid on
for i=1:np
    hold on
    grid on
    scatter3(teleYZ(1,i),teleYZ(2,i),teleYZ(3,i),w,'r');
end
axis equal


%%
telepYZ=teleYZ';
[noYZ,VYZ,pYZ]=affine_fit(telepYZ);

% How to generate and plot the plane

dYZ=-pYZ*noYZ;
% [xx,yy]=ndgrid(0.02:0.10,0.02:0.10);
% [xx,yy]=ndgrid(-0.02:-0.01:-0.12,-0.02:-0.01:-0.10);
[xxYZ,yyYZ]=ndgrid(0.05:-0.01:-0.06,0.03:-0.01:-0.10);

figure(2)
hold on
zYZ=(-noYZ(1)*xxYZ-noYZ(2)*yyYZ-dYZ)/noYZ(3);
suYZ=surf(xxYZ,yyYZ,zYZ);
suYZ.EdgeColor='none';
suYZ.FaceColor=[1 0 0];
suYZ.FaceAlpha=0.3;

%%  calculating the distance of each point from the plane
aYZ=noYZ(1);
bYZ=noYZ(2);
cYZ=noYZ(3);
max=np;
for i=1:max

    vYZ(:,i)= point_plane_shortest_dist_vec(teleYZ(1,i),teleYZ(2,i),teleYZ(3,i), aYZ, bYZ, cYZ, dYZ);
    distYZ(:,i)=norm(vYZ(:,i));
end


err_distYZ=mean(distYZ);
std_errYZ=std(distYZ);








