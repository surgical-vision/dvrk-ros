%% This is a function useful for setting the robot state into CARTESIAN_GOAL



function [ready]=cartesian_state(r)

set_state(r, 'DVRK_POSITION_GOAL_JOINT') ;
set_state(r, 'DVRK_POSITION_GOAL_CARTESIAN');

robotstate=r.robot_state;
jointstate='DVRK_POSITION_GOAL_CARTESIAN';

% Check if it reaches the state
if strcmp(robotstate,jointstate)==1
    ready=1;
else
    ready=0;
end


end 