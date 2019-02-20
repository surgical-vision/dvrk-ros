

function readCallBackJOINT(~,message)

    global Joint
    
    Joint = [message.Position];

end