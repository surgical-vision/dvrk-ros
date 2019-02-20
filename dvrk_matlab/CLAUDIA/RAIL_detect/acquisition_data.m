%% Acquiring and plotting the joint 


function [joint_motion,cartesian_motion]=acquisition_data(r) 

nmax=200000;

for i =1:nmax
    
    joint_motion(:,i)=get_state_joint_current(r);
    cartesian_motion(:,:,i)=get_position_current(r);
    
    
end

t=[1:nmax];


% figure()
% title('Plotting joint value')
% scatter(joint_motion(1,:),t,'r','filled');
% hold on
% scatter(joint_motion(2,:),t,'g','filled');
% hold on
% scatter(joint_motion(3,:),t,'b','filled');
% hold on
% scatter(joint_motion(4,:),t,'y','filled');
% hold on
% scatter(joint_motion(5,:),t,'m','filled');
% hold on
% scatter(joint_motion(6,:),t,'c','filled');
% legend('joint1','joint2','joint3','joint4','joint5','joint6');
% 
% 
% figure()
% title('Plotting reference frames')
% trplot(cartesian_motion(:,:,:), '3d');


end