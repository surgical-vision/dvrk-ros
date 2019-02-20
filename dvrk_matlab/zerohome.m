%% Setting the home reference position

% This position is not the real home position of the arm, it's a zero
% position useful for setting the state of the machine to the
% cartesian_state, cause we need the tool to be inserted inside the
% canuala, otherwise we cannot set the cartesian_state. I decide to set the
% tool at halfh of the entire motion field



function [reach,pos_curr0,pos_des0] = zerohome(r)



set_state(r, 'DVRK_POSITION_GOAL_JOINT')     

% It's important to move the joint from the last to the first cause
% otherwise the tip tool would not be able to enter insie the canula
i=7;
while (i>0)
    r.move_joint_one(0.0, int8(i));
    i=i-1;
    
end


set_state(r, 'DVRK_POSITION_GOAL_JOINT')
r.move_joint_one(0.12, int8(3)); % this set the tool

pos_curr0=r.position_current;
pos_des0=r.position_desired;

robotstate=r.robot_state;
jointstate='DVRK_POSITION_GOAL_JOINT';

%Check if the state has been reached
if strcmp(robotstate,jointstate)==1
    reach=1;
else
    reach=0;
end

reach


end
