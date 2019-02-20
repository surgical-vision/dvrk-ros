function cos_dir=createRF(P1,P2,P3,P4)

xf=P2-P1;
xf=xf/norm(xf);

y_temp=P3-P1;
y_temp=y_temp/norm(y_temp);

zf=cross(xf,y_temp);
zf=zf/norm(zf);

yf=cross(zf,xf);
cos_dir=[xf yf zf];

if nargin>3
    Tf=r2t([xf yf zf]);
    Tf(1:3,4)=P4;
    cos_dir=Tf;
end


end