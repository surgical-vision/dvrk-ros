%% Error table generation

%% Motion of the two arm
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

rosinit
rostopic list
rosnode list

% From QtGui set home and then start

%% We create a interface with ROS
% 
r1= arm('PSM1');
r2 = arm('PSM2');

% Remember for setting the  home position the tool does not have to be
% inserted. After 4-5 seconds insert the tool and verify the robot state
r1.home() 
r1

r2.home() 
r2


%% Setting zero-position
[reach,pos_curr0,pos_des0] = zerohome(r1);   % PSM1
[reach,pos_curr0,pos_des0] = zerohome(r2);  % PSM2

%% Setting cartesian state 

[ready1]=cartesian_state(r1);
r1


[ready2]=cartesian_state(r2);
r2
%% Movement of the EE
% x=1
% y=2
% z=3

axe=2;

if axe==3
    for i=1:8
       

        if (i<5)
        p= [0.0 0.0 (i*0.005)];
        dmove_translation(r1, p)
        dmove_translation(r2, p)
        positionCartZ1(:,:,i)=r1.position_current();
        positionCartZ2(:,:,i)=r2.position_current();
        positionCartZ_ideal1(:,:,i)=r1.position_desired();
        positionCartZ_ideal2(:,:,i)=r2.position_desired();
        positionJointZ1(:,i)=r1.position_joint_current();
        positionJointZ2(:,i)=r2.position_joint_current();
        positionJointZ_ideal1(:,i)=r1.position_joint_desired();
        positionJointZ_ideal2(:,i)=r2.position_joint_desired();
        [reach1,pos_curr01,pos_des01] = zerohome(r1);
        [reach2,pos_curr02,pos_des02] = zerohome(r2);
        [ready1]=cartesian_state(r1);
        [ready2]=cartesian_state(r2);

        else 
        k=i-4;
        p= [0.0 0.0 -(k*0.005)];
        dmove_translation(r1, p)
        dmove_translation(r2, p)
        positionCartZ1(:,:,i)=r1.position_current();
        positionCartZ2(:,:,i)=r2.position_current();
        positionCartZ_ideal1(:,:,i)=r1.position_desired();
        positionCartZ_ideal2(:,:,i)=r2.position_desired();
        positionJointZ1(:,i)=r1.position_joint_current();
        positionJointZ2(:,i)=r2.position_joint_current();
        positionJointZ_ideal1(:,i)=r1.position_joint_desired();
        positionJointZ_ideal2(:,i)=r2.position_joint_desired();
        [reach1,pos_curr01,pos_des01] = zerohome(r1);
        [reach2,pos_curr02,pos_des02] = zerohome(r2);
        [ready1]=cartesian_state(r1);
        [ready2]=cartesian_state(r2);
        end
        rosinit
    end
    
elseif axe==2
 
    for i=1:8
        for j=1:2

        if (i<5)
        p= [0.0 (i*0.005) 0.0];
        dmove_translation(r1, p)
        dmove_translation(r2, p)
        positionCartY1(:,:,i)=r1.position_current();
        positionCartY2(:,:,i)=r2.position_current();
        positionCartY_ideal1(:,:,i)=r1.position_desired();
        positionCartY_ideal2(:,:,i)=r2.position_desired();
        positionJointY1(:,i)=r1.position_joint_current();
        positionJointY2(:,i)=r2.position_joint_current();
        positipositionJointY_ideal1(:,i)=r1.position_joint_desired();
        positipositionJointY_ideal2(:,i)=r2.position_joint_desired();
        [reach1,pos_curr01,pos_des01] = zerohome(r1);
        [reach2,pos_curr02,pos_des02] = zerohome(r2);
        [ready1]=cartesian_state(r1);
        [ready2]=cartesian_state(r2);

        else 
        k=i-4;
        p= [0.0 -(k*0.005) 0.0];
        dmove_translation(r1, p)
        dmove_translation(r2, p)
        positionCartY1(:,:,i)=r1.position_current();
        positionCartY2(:,:,i)=r2.position_current();
        positionCartY_ideal1(:,:,i)=r1.position_desired();
        positionCartY_ideal2(:,:,i)=r2.position_desired();
        positionJointY1(:,i)=r1.position_joint_current();
        positionJointY2(:,i)=r2.position_joint_current();
        positionJointY_ideal1(:,i)=r1.position_joint_desired();
        positipositionJointY_ideal2(:,i)=r2.position_joint_desired();
        [reach1,pos_curr01,pos_des01] = zerohome(r1);
        [reach2,pos_curr02,pos_des02] = zerohome(r2);
        [read1]=cartesian_state(r1);
        [ready2]=cartesian_state(r2);
        
        end
        end
    end
    
elseif axe==1
    for i=1:8

        if (i<5)
        p= [(i*0.005) 0.0 0.0];
        dmove_translation(r, p)
        positionCartX(:,:,i)=r.position_current();
        positionCartX_ideal(:,:,i)=r.position_desired();
        positionJointX(:,i)=r.position_joint_current();
        positionJointX_ideal(:,i)=r.position_joint_desired();
        [reach,pos_curr0,pos_des0] = zerohome(r);
        [ready]=cartesian_state(r);

        else 
        k=i-4;
        p= [-(k*0.005) 0.0 0.0];
        dmove_translation(r, p)
        positionCartX(:,:,i)=r.position_current();
        positionCartX_ideal(:,:,i)=r.position_desired();
        positionJointX(:,i)=r.position_joint_current();
        positionJointX_ideal(:,i)=r.position_joint_desired();
        [reach,pos_curr0,pos_des0] = zerohome(r);
        [ready]=cartesian_state(r);

        end
    end
end




