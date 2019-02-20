%% How to controll the joints' arm
% 
% You can find usefull information inside the readme file in 
% /home/davinci/catkin_ws/src/dvrk-ros/dvrk_matlab
% 
% How to controll the robot joint through matlab interface
% 
% 1. Start the application (in the terminal): 
%   rosrun dvrk_robot dvrk_console_json -j /home/davinci/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/ucl-daVinci/console-PSM1.json
% 2. In a new terminal power on the arm with the command: qladisp n1 n2 and
%   press [p] to power the arm. Check in the dvrk window if the arm is
%   correctly powered.
% 3. Go to matlab ... run the comand step by step
%       More info: https://uk.mathworks.com/help/robotics/robot-operating-system-ros.html

%%  To initialize ros
% It's used to start the global ROS node with a default MATLAB® name and tries 
% to connect to a ROS master running on localhost and port 11311. If the global 
% ROS node cannot connect to the ROS master, rosinit also starts a ROS core in MATLAB, 
% which consists of a ROS master, a ROS parameter server, and a rosout logging node.

rosinit


% (vuol dire che se non ho un terminale con attivo roscore, lo attiva direttamente 
% questo comando in matlba). 

%% To check if it is corretly working
rostopic list
%% Creating the robot arm model through the arm.m function 
r = arm('PSM1');

%% To controll the robot -> numers ar radiant and meters {3}. 

r.home()% reaching home position(??? not sure if it's working)

%%
r.dmove_joint_one(-0.098, int8(7))   % to move a single joint (insert valure, insert number of joint {1 to 7})

%%

r.dmove_joint([-0.01, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0])  % to move all joints

%%
% Shoul allow to move in cartesian space 
v = [0.0, 0.0, 0.054];
r.dmove_translation(v)



