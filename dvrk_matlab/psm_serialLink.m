%% Try to model PSM
close all
clear all
clc
%%
% Definition of the DH parameter 

L(1)=Link('a',0,'d',0,'alpha',pi/2,'offset',pi/2,'modified');
L(1)=Link('a',0,'d',0,'alpha',-pi/2,'offset',-pi/2,'modified');
L(3)=Link('a',0,'alpha',pi/2,'theta',0,'offset', 0.4318, 'modified');
L(4)=Link('a',0,'d',0.4162,'alpha',0,'modified' );
L(5)=Link('a',0,'d',0,'alpha',-pi/2,'offset',-pi/2,'modified' );
L(6)=Link('a',0.001,'d',0,'alpha',-pi/2,'offset',-pi/2, 'modified');
L(7)=Link('a',0,'d',0.0102,'alpha',-pi/2,'modified' );

% L(3).qlim=[0 0.024];
% ws=[-1,1,-1,1,-1,1];

plotOptions={'workspace',ws};

PSM=SerialLink(L,'name','PSM1','plotopt',plotOptions);

disp(PSM)

%%
figure(2)
PSM.teach([0 0 0 0 0 0 0])


