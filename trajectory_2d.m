clear all
close all
clc

% start
vmax=0.5; amax=2; v_nodo=0.5*vmax;
approx1=floor((3*pi/2)*100);
approx2=floor(3*pi*100);
approx3=floor((3*pi/2)*100);
tot_punti=approx1+approx2+approx3-2;
xl=zeros(1,tot_punti); yl=zeros(1,tot_punti); zl=zeros(1,tot_punti);

sf=0.13;

% primo tratto circolare
tt=[]; s=[]; v=[]; a=[]; vx=[]; vy=[]; ax=[]; ay=[];
[R,xcyc]=fit_circle_through_3_points([0 3; -1 2; 0 1]);
xc=xcyc(1)*sf; yc=xcyc(2)*sf;
R=sf*R;
th=linspace(0,3*pi/2,approx1); 
xl1=R*cos(th)+xc;
xl(1:approx1)=xl1;
yl1=R*sin(th)+yc;
yl(1:approx1)=yl1;
[tt1,s1,v1,a1,vx1,vy1,ax1,ay1,alfa] = look_ahead1(xl1,yl1,0,0,0,v_nodo,vmax*0.8,amax,approx1,xc,yc,R,1);
tt=[tt1]; s=[s1]; v=[v1]; a=[a1]; vx=[vx1]; vy=[vy1]; ax=[ax1]; ay=[ay1];

 % secondo tratto circolare
[R,xcyc]=fit_circle_through_3_points([0 1; 1 0; 0 -1]);
xc=xcyc(1)*sf; yc=xcyc(2)*sf;
R=sf*R;
th=linspace(pi/2,-2*pi-pi/2,approx2);
xl2=R*cos(th)+xc;
xl(approx1:approx1+approx2-1)=xl2;
yl2=R*sin(th)+yc;
yl(approx1:approx1+approx2-1)=yl2;
[tt2,s2,v2,a2,vx2,vy2,ax2,ay2,alfa] = look_ahead1(xl2,yl2,tt(end),s(end),v(end),0,vmax*0.8,amax,approx2,xc,yc,R,2);
tt=[tt tt2(2:end)]; s=[s s2(2:end)]; v=[v v2(2:end)] ; a=[a a2(2:end)]; vx=[vx vx2(2:end)]; vy=[vy vy2(2:end)]; ax=[ax ax2(2:end)]; ay=[ay ay2(2:end)];

 % terzo tratto circolare
[R,xcyc]=fit_circle_through_3_points([0 -1; 1 -2; 0 -3]);
xc=xcyc(1)*sf; yc=xcyc(2)*sf;
R=sf*R;
th=linspace(pi/2,-pi,approx3);
xl3=R*cos(th)+xc;
xl(approx1+approx2-1:approx1+approx2+approx3-2)=xl3;
yl3=R*sin(th)+yc;
yl(approx1+approx2-1:approx1+approx2+approx3-2)=yl3;
[tt3,s3,v3,a3,vx3,vy3,ax3,ay3,alfa] = look_ahead1(xl3,yl3,tt(end),s(end),v(end),0,vmax*0.8,amax,approx3,xc,yc,R,2);
tt=[tt tt3(2:end)]; s=[s s3(2:end)]; v=[v v3(2:end)] ; a=[a a3(2:end)]; vx=[vx vx3(2:end)]; vy=[vy vy3(2:end)]; ax=[ax ax3(2:end)]; ay=[ay ay3(2:end)];

% rotazione in 3d
zl=zeros(1,length(xl));
vz=zeros(1,length(vx));
az=zeros(1,length(ax));
[x,y,z] = Rotation3D_MAAT(xl,yl,zl,pi/2.7,pi/8,-pi/6);
[vx,vy,vz] = Rotation3D_MAAT(vx,vy,vz,pi/2.7,pi/8,-pi/6);
[ax,ay,az] = Rotation3D_MAAT(ax,ay,az,pi/2.7,pi/8,-pi/6);

% traslazione
x=x+0.5; y=y+1.15; z=z+0.75;

% tratto iniziale e finale
approxi=floor((((x(1)-0.3)^2 +(y(1)-(0.5+0.1+0.5))^2 +(z(1)-0.8)^2)^0.5)*1000);
xi=linspace(0.3,x(1),approxi);
yi=linspace(0.5+0.1+0.5,y(1),approxi);
zi=linspace(0.8,z(1),approxi);
[tt_ti,s_ti,v_ti,a_ti,vx_ti,vy_ti,vz_ti,ax_ti,ay_ti,az_ti] = look_ahead(xi,yi,zi,0,0,0,0,vmax,amax,approxi);
approxf=floor((((x(end)-0.3)^2 +(y(end)-(0.5+0.1+0.5))^2 +(z(end)-0.8)^2)^0.5)*500);
xf=linspace(x(end),0.3,approxf);
yf=linspace(y(end),0.5+0.1+0.5,approxf);
zf=linspace(z(end),0.8,approxf); 
[tt_tf,s_tf,v_tf,a_tf,vx_tf,vy_tf,vz_tf,ax_tf,ay_tf,az_tf] = look_ahead(xf,yf,zf,tt(end),s(end),v(end),0,vmax,amax,approxf);

tt=[tt_ti(1:end) tt+tt_ti(end) tt_tf(2:end)+tt_ti(end)]; s=[s_ti(1:end) s+s_ti(end) s_tf(2:end)+s_ti(end)]; v=[v_ti(1:end) v v_tf(2:end)]; a=[a_ti(1:end) a a_tf(2:end)];
vx=[vx_ti(1:end) vx vx_tf(2:end)]; vy=[vy_ti(1:end) vy vy_tf(2:end)]; vz=[vz_ti(1:end) vz vz_tf(2:end)];  ax=[ax_ti(1:end) ax ax_tf(2:end)]; ay=[ay_ti(1:end) ay ay_tf(2:end)]; az=[az_ti(1:end) az az_tf(2:end)];
x=[xi(1:end) x xf(2:end)]; y=[yi(1:end) y yf(2:end)]; z=[zi(1:end) z zf(2:end)];

figure
plot(tt,v,tt,a)
legend('v','a')
title('gripper kinematic to t')

figure
plot(s,v,s,a)
legend('v','a')
title('gripper kinematic to s')

figure
vdx=diff(x)./diff(tt);
vdy=diff(y)./diff(tt);
plot(tt,vy,tt(1:end-1),vdy)
legend('v','debug')
title('debug v')

figure
plot(tt,ax,tt,ay,tt,az)
legend('ax','ax','az')
title('gripper acceleration components')

figure
plot(tt,vx,tt,vy,tt,vz)
legend('vx','vy','vz')
title('gripper velocity components')

figure
plot(tt,s,tt,v,tt,a)
legend('s','v','a')
title('gripper kinematic')

figure
plot(tt,x,'.-',tt,y,'.-',tt,z,'.-')
legend('x','y','z')
title('gripper position')