function readCallbackR(~, message)
   
    % Declare global variables to store position and orientation
    global posR
    %global orient
    
    % Extract position from the ROS message and assign the
    % data to the global variables.
    posR = [message.X message.Y message.Z];
    %orient = [message.Angular.X message.Angular.Y message.Angular.Z];
end