%% extract the values saved with rosbag - JOINT VALUES
% Command to type in the command window 
%       cd bagfiles
%       rosbag record /dvrk/PSM1/state_joint_current
filepathj = fullfile('/home/davinci/bagfiles','RJ6.bag');
bagj=rosbag(filepathj);
msgj=readMessages(bagj);
showdetails(msgj{1})
% Extracting the joint position values
joint_bag=cellfun(@(m) (m.Position), msgj,'un',0);
%saving the values in a matrix
for i = 1:length(joint_bag)
   
    joint_plot(:,i)=joint_bag{i};
    
end
% plotting the value obatined
figure(987)
hold on
title('joint values')
plot([1:1:size(joint_plot,2)],joint_plot(:,1:size(joint_plot,2)))
grid on

% Plotting each joint separately
figure(645)
%title('Representation of each joint value')
subplot(2,3,1)
hold on
plot([1:1:size(joint_plot,2)],joint_plot(1,1:size(joint_plot,2)))
grid on
title('Joint-1 - [deg]')
hold on
subplot(2,3,2)
hold on
plot([1:1:size(joint_plot,2)],joint_plot(2,1:size(joint_plot,2)))
grid on    
title('Joint-2 - [deg]')
subplot(2,3,3)
hold on
plot([1:1:size(joint_plot,2)],joint_plot(3,1:size(joint_plot,2)))
grid on9999999999999999999999
title('Joint-3 - [m]')
subplot(2,3,4)
hold on
plot([1:1:size(joint_plot,2)],joint_plot(4,1:size(joint_plot,2)))
grid on
title('Joint-4 - [deg]')
subplot(2,3,5)
hold on
plot([1:1:size(joint_plot,2)],joint_plot(5,1:size(joint_plot,2)))
grid on9999999999999999999999
title('Joint-5 - [deg]')
subplot(2,3,6)
hold on
plot([1:1:size(joint_plot,2)],joint_plot(6,1:size(joint_plot,2)))
grid on
title('Joint-6 - [deg]')

9999
%% Recording and extracting CARTESIAN VALUES 
%       cd bagfiles
%       rosbag record /dvrk/PSM1/position_cartesian_current
filepathc = fullfile('/home/davinci/bagfiles','RC3.bag');
bagc=rosbag(filepathc);
msgc=readMessages(bagc);
showdetails(msgc{1});

% Extracting the joint position values
cartPos_bag=cellfun(@(m) (m.Pose.Position), msgc,'un',0);
cartOri_bag=cellfun(@(m) (m.Pose.Orientation), msgc,'un',0);

%saving the values in a matrix
T_cart=zeros(4);
T_cart(4,4)=1;
for i = 1:length(cartPos_bag)
   
    T_cart(1,4,i)=cartPos_bag{i}.X;
    T_cart(2,4,i)=cartPos_bag{i}.Y;
    T_cart(3,4,i)=cartPos_bag{i}.Z;% plotting the value obatined
    T_cart(4,4,i)=1;
    %saving the quaternion
    quat(i,1)=cartOri_bag{i}.X;
    quat(i,2)=cartOri_bag{i}.Y;
    quat(i,3)=cartOri_bag{i}.Z;
    quat(i,4)=cartOri_bag{i}.W;
    %trasformin in rot matrix
    T_cart(1:3,1:3,i)=quat2rotm(quat(i,:));
    posizione(1:3,i,2)= T_cart(1:3,4,i);

end
%%
% figure()
% title('Plotting cartesian position')
% for i =1:5 %length(cartPos_bag)
s=3;
figure(912)
hold on
title('cartesian position')
scatter3(T_cart(1,4,:),T_cart(2,4,:),T_cart(3,4,:),s)
grid on
% trplot(T_cart(:,:,i),'3d')
% hold on
% end
% grid on

%


999


