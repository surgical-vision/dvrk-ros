%% PSM MOTION: validation of the accuracy of the PSM motion 

% Definition of the trajectory points 
square=0.010;    % if i want m (size of the chessboard )
for d=1
point(1:3,1)=[0*square; 8*square; 5*square];
point(1:3,2)=[1*square; 7*square; 5*square];
point(1:3,3)=[4*square; 4*square; 6*square];
point(1:3,4)=[2*square; 2*square; 2*square];
point(1:3,5)=[5*square; 5*square; 3*square];
point(1:3,6)=[6*square; 2*square; 0*square];
% point(1:3,7)=[3*square; 3*square; 2*square];
% point(1:3,8)=[4*square; 4*square; 2*square];
% point(1:3,9)=[4*square; 4*square; 1*square];
end 

%% Plotting points move_translation(r,home_psm(1:3,1)')

point(4,:)=ones();
for i=1:size(point,2)
    point_psm(:,i)=T_ws_to_rc*point(:,i);
    figure(1)   % if i want the same picture of the ws calibration otherwise 
%     figure(2)
    hold on
    scatter3(point(1,i),point(2,i), point(3,i),'r','filled')
    hold on
    scatter3(point_psm(1,i),point_psm(2,i),point_psm(3,i),'b','filled')
    hold on
end
grid on

%% Home reference position for PSM motion 
square= 8.73/1000;
home(1:3,1,1)=[0*square;0*square;-7*square];
home(4,:)=ones();
home_psm(:,1)=T_ws_to_rc*home(:,1);
move_translation(r,home_psm(1:3,1)')
%%
move_translation(r,[0.1142 0.0093 -0.1457])

%% Motion along the traj of points
pointc=point_psm(1:3,:);
home_psm=home_psm(1:3,:);
%%
move_translation(r,home_psm(1:3,1)')
%%
for i=1:size(pointc,2)
    move_translation(r,pointc(:,i)')
    pause(0.5)
end



%% PROVA CON ALTRA HOME
clearvars home0_psm 
home0(1:3,1,1)=[1*square;1*square;0*square];
home0(4,:)=ones();
home0_psm(:,1)=T_ws_to_rc*home0(:,1);
home0_psm=home0_psm(1:3,:);

%%
move_translation(r,home0_psm(:,1)')


