count=1;
numpoint= 29; % num of points acquired for the acquisition + 1 

while (count<numpoint)
    
     c1=r.get_state_joint_current;
     cjoint(:,:,count)=c1(1,1);   % generation of the matrix of joint value

     c1c=r.get_position_current();
     cc_weiss(:,:,count)=c1c;    
    
    while ~waitforbuttonpress 
                    
    end                                
      count=count+1;
      close
end

%%
np=numpoint;

for i=1:np-1
    move_weiss(:,i)=cc_weiss(1:3,4,i);
end
%%

move_weiss=move_weiss(:,1:26);
%%
pause(5)
for i =1:size(move_weiss,2)
    move_translation(r,move_weiss(1:3,i)')
    pause(0.1)
end

%%
home(1:3,1,1)=[1*square;1*square;-5*square];
home(4,:)=ones();
home_psm(:,1)=T_ws_to_rc*home(:,1);
move_translation(r,home_psm(1:3,1)')

