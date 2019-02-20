function readCallbackL(~, message)
   
    % Declare global variables to store position and orientation
    global posL
    %global orient
    
    % Extract position from the ROS message and assign the
    % data to the global variables.
    posL = [message.X message.Y message.Z];
    %orient = [message.Angular.X message.Angular.Y message.Angular.Z];
end