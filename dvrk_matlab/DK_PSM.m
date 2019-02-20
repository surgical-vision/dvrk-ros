%% DIRECT KINEMATIC PSM (joint 1 to 7)

% This script generate all the transformation matrix implied into the
% direct kinematic of the PSM arm. It can show all the transformation
% matrix implied in the chain or even the transformation among the desired
% joint.
% Moreover is possible to assigne the desired joint value and determine the
% effective value of the transformation.

close all 
clear all
clc

%% 
% Definition of symbolic variables for calculatin the matrix

syms q1 q2 q3 q4 q5 q6  %joint value

% known length of the arm
l1=sym('l_RCC'); % l_RCC=0.4318 m
l2=sym('l_tool'); %l_tool=0.4162 m
l3=sym('l_P2Y'); % l_P2Y= 0.0091 m
l4=sym('l_Y2C'); % l_Y2C= 0.0102 m


% Definition of the DH parameter 
a=[0 0 0 0 0 l3 0];
alpha=[pi/2 -pi/2 pi/2 0 -pi/2 -pi/2 -pi/2];
d=[0 0 (q3-l1) l2 0 0 l4];
theta=[(q1+pi/2) (q2-pi/2) 0 q4 (q5-pi/2) (q6-pi/2) 0];

%%
% Creation of the 6 transformation matrix involeved in the DK chain 

for k=1:7

    A(:,:,k)=[cos(theta(k)) (-cos(alpha(k))*sin(theta(k))) (sin(alpha(k))*sin(theta(k)))   a(k)*cos(theta(k));
              sin(theta(k)) (cos(alpha(k))*cos(theta(k)))  (-sin(alpha(k))*cos(theta(k)))  a(k)*sin(theta(k));
              0             sin(alpha(k))                   cos(alpha(k))                  d(k);
              0             0                               0                              1];
    
end

%%
% Refinement of the layout of the transformation matrix

A(1:2,2,1)=0;
A(3,3,1)=0;
A(1:2,2,2)=0;
A(3,3,2)=0;
A(1:2,2,3)=0;
A(2:3,1,3)=0;
A(1,3,3)=0;
A(3,3,3)=0;

A(3,3,5)=0;
A(1:2,2,5)=0;
A(3,3,6)=0;
A(1:2,2,6)=0;
A(3,3,7)=0;
A(2,2,7)=0;


% Substitution of the sin and cos +-pi/2

for k=1:7
    for i=1:4
        for j=1:4
            if (A(i,j,k)==cos(q1 + pi/2))
                A(i,j,k)=-sin(q1);
                
                elseif (A(i,j,k)==-cos(q2 + pi/2))
                    A(i,j,k)=+sin(q2);
                elseif (A(i,j,k)==-cos(q1 + pi/2))
                    A(i,j,k)=+sin(q1);
                    
            elseif (A(i,j,k)==cos(q2 + pi/2))
                    A(i,j,k)=-sin(q2);
            elseif (A(i,j,k)==cos(q3 + pi/2))
                    A(i,j,k)=-sin(q3);
            elseif (A(i,j,k)==cos(q4 + pi/2))
                    A(i,j,k)=-sin(q4);        
            elseif (A(i,j,k)==cos(q5 + pi/2))
                    A(i,j,k)=-sin(q5);
            elseif (A(i,j,k)==cos(q6 + pi/2))
                    A(i,j,k)=-sin(q6); 
                   
            elseif (A(i,j,k)==sin(q1 + pi/2))
                    A(i,j,k)=cos(q1);
            elseif (A(i,j,k)==sin(q2 + pi/2))
                    A(i,j,k)=cos(q2);
            elseif (A(i,j,k)==sin(q3 + pi/2))
                    A(i,j,k)=cos(q3);
            elseif (A(i,j,k)==sin(q4 + pi/2))
                    A(i,j,k)=cos(q4);
            elseif (A(i,j,k)==sin(q5 + pi/2))
                    A(i,j,k)=cos(q5);
            elseif (A(i,j,k)==sin(q6 + pi/2))
                    A(i,j,k)=cos(q6);

             elseif (A(i,j,k)==sin(q1 - pi/2))
                    A(i,j,k)=-cos(q1);        
             elseif (A(i,j,k)==sin(q2 - pi/2))
                    A(i,j,k)=-cos(q2);
                    
                    elseif (A(i,j,k)==-sin(q2 - pi/2))
                    A(i,j,k)=cos(q2);
                    
             elseif (A(i,j,k)==sin(q3 - pi/2))
                    A(i,j,k)=-cos(q3);
             elseif (A(i,j,k)==sin(q4 - pi/2))
                    A(i,j,k)=-cos(q4);
             elseif (A(i,j,k)==sin(q5 - pi/2))
                    A(i,j,k)=-cos(q5);
                    
                    elseif (A(i,j,k)==-sin(q5 - pi/2))
                    A(i,j,k)=cos(q5);
                    
             elseif (A(i,j,k)==sin(q6 - pi/2))
                    A(i,j,k)=-cos(q6);
           
                    elseif (A(i,j,k)==-sin(q6 - pi/2))
                    A(i,j,k)=+cos(q6);
                    
            elseif (A(i,j,k)==cos(q1 - pi/2))
                    A(i,j,k)=sin(q1);
            elseif (A(i,j,k)==cos(q2 - pi/2))
                    A(i,j,k)=sin(q2);
            elseif (A(i,j,k)==cos(q3 - pi/2))
                    A(i,j,k)=sin(q3);
            elseif (A(i,j,k)==cos(q4 - pi/2))
                    A(i,j,k)=sin(q4);
            elseif (A(i,j,k)==cos(q5 - pi/2))
                    A(i,j,k)=sin(q5);
            elseif (A(i,j,k)==cos(q6 - pi/2))
                    A(i,j,k)=sin(q6);
                    
                    elseif (A(i,j,k)==l3*cos(q6 - pi/2))
                    A(i,j,k)=l3*sin(q6);
                    elseif (A(i,j,k)==l3*sin(q6 - pi/2))
                    A(i,j,k)=-l3*cos(q6);
            end
        end
    end
end


%%
% Creation of all the intermediate matrixs 

A1=A(:,:,1);
A2=A(:,:,2);
A3=A(:,:,3);
A4=A(:,:,4);
A5=A(:,:,5);
A6=A(:,:,6);
A7=A(:,:,7);



A02(:,:)=A1*A2;  %from 0 to 2
A03(:,:)=A02*A3; %from 0 to 3
A04(:,:)=A03*A4; %from 0 to 4
A05(:,:)=A04*A5; %from 0 to 5
A06(:,:)=A05*A6; %from 0 to 6
A07(:,:)=A06*A7; %from 0 to 7 -> Entire Direct Kinematic chain 

A45(:,:)=A4*A5; % from 4 to 5
A67(:,:)=A6*A7; %from 6 to 7
A47=A45*A67; %from 4 to 7


%%
% Transforming all the matrix with the handle @ notation 

% Single transformation 


A1h=matlabFunction(A1);
A2h=matlabFunction(A2);
A3h=matlabFunction(A3);
A4h=matlabFunction(A4);
A5h=matlabFunction(A5);
A6h=matlabFunction(A6);
A7h=matlabFunction(A7);

% If we want to assign the value of joint value 

% Assign q value to vector q=[] and the length of the segment 

l1=0.4318; %l_RCC
l2=0.4162; %l_tool
l3= 0.0091; %l_P2Y
l4= 0.0102; %l_Y2C
 
q=[2 3 4 5 6 7]; % Assign the 6 joint value here

A1_joint=A1h(q(1));
A2_joint=A2h(q(2));
A3_joint=A3h(l1,q(3));
A4_joint=A4h(l2,q(4));
A5_joint=A5h(q(5));
A6_joint=A6h(l3,q(6));
A7_joint=A7h(l4);

A_joint(:,:,1)=A1_joint;
A_joint(:,:,2)=A2_joint;
A_joint(:,:,3)=A3_joint;
A_joint(:,:,4)=A4_joint;
A_joint(:,:,5)=A5_joint;
A_joint(:,:,6)=A6_joint;
A_joint(:,:,7)=A7_joint;


%%
% Creation of the matrix with the joint value assigned 

for k=1:6
    
    Aint_joint(:,:,k)=A_joint(:,:,k)*A_joint(:,:,k+1);
end
    
A02_joint(:,:)=Aint_joint(:,:,1);  %from 0 to 2
A03_joint(:,:)=Aint_joint(:,:,2); %from 0 to 3
A04_joint(:,:)=Aint_joint(:,:,3); %from 0 to 4
A05_joint(:,:)=Aint_joint(:,:,4); %from 0 to 5
A06_joint(:,:)=Aint_joint(:,:,5); %from 0 to 6
A07_joint(:,:)=Aint_joint(:,:,6); %from 0 to 7 -> Entire Direct Kinematic chain 



