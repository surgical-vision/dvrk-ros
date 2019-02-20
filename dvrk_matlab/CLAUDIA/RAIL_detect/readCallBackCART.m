

function readCallBackCART(~,message)
    
    global Position
    global Orientation
    
    Position = [message.Pose.Position];
    Orientation = [message.Pose.Orientation];
    



end