%% Surface scanning for kidney registration

max=200;

% press a button and the acquisition will start after 4 seconds (start the scanning procedure)
while ~waitforbuttonpress 
                    
end 
pause(4)

for i=1:max
    cK=r.get_state_joint_current;
    cKjoint(:,:,i)=cK;  
    cKc=r.get_position_current();
    ccK(:,:,i)=cKc;  
    pause(0.3)
end 

save('ccKidSurf','ccK');
save('ccKidjoint','cKjoint');

%%
load('cc');
load('cjointXY');
%%
teleopKID=ccK;
np=size(ccK,3);

for i=1:np
    teleKID(:,i)=teleopKID(1:3,4,i); 
end

%% Using fitting toolbox

teleKID_X=teleKID(1,:);
teleKID_Y=teleKID(2,:);
teleKID_Z=teleKID(3,:);


%%
W=repmat([50],1,1);
w=W(:);
figure()
hold on
grid on
for i=1:np
    hold on
    grid on
    scatter3(teleKID(1,i),teleKID(2,i),teleKID(3,i),w,'b');
end
axis equal


