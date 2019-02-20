%% Definizione of a node for acquiring data

node1 = robotics.ros.Node('/PSM_data_acquisition');
% CART_pos = robotics.ros.Subscriber(node1,'/dvrk/PSM1/position_cartesian_current','geometry_msgs/PoseStamped',@readCallBackCART);
JOINT_pos = robotics.ros.Subscriber(node1,'/dvrk/PSM1/state_joint_current','sensor_msgs/JointState',@readCallBackJOINT);

% global Position
% global Orientation
global Joint


while (count < nmax)
        
    joint_val()=Joint
    
end




% CART = receive(CART_pos,10);
% JOINT = receive(JOINT_pos,10);


