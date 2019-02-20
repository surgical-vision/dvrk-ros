%% Needle grasp

A=[(square*2.5);(square*1.5);0];

%% Point Home
ch=r.position_joint_current;
cjoint(:,:,1)=ch;

chc=r.position_current(:,:);

%% Point needle
cn=r.position_joint_current;
cjoint(:,:,1)=cn;

cnc=r.position_current(:,:);

%% Point open
cO=r.position_joint_current;
cjoint(:,:,1)=cO;

cOc=r.position_current(:,:);

%% target

move(r,cnc)
%% Open
dmove_joint_one(r,cO(7),int8(7))

%% home position 

move(r,chc)
%% close grip
dmove_joint_one(r,cn(7),int8(7))

%% grasp
move(r,cnc)
pause(0.5)
dmove_joint_one(r,cn(7),int8(7))
dmove_joint_one(r,cn(7),int8(7))
pause(0.5)
move(r,chc)
pause(2)
move(r,cnc)
pause(0.5)
dmove_joint_one(r,cO(7),int8(7))
dmove_joint_one(r,cO(7),int8(7))
pause(0.3)
move(r,chc)
