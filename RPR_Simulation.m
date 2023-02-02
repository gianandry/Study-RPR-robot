% Project Robot RPR with trajectory of the letter imposed on s

% clear all
close all
clc

% Link lengths
l1=0.8; % link 1
l2=0.3; % link 2
l3=0.5; % link 3
l4=0.6; % link 4
l5=0.5; % link 5
Length=[l1, l2, l3, l4, l5]';
nlink=3; % number of mobile links of the robot

% limiti attuatori
qp1max=0.5; qpp1max=1.5;
qp2max=0.5; qpp2max=1.5;
qp3max=1; qpp3max=4;
qpmax=[qp1max qp2max qp3max];
qppmax=[qpp1max qpp2max qpp3max];

% limiti percorso Look Ahead
vmax=0.5; amax=1; v_nodo=0.5*vmax;

% trajectory planning
approx1=floor((3*pi/2)*200);
approx2=floor(3*pi*200);
approx3=floor((3*pi/2)*200);
tot_punti=approx1+approx2+approx3-2;
xl=zeros(1,tot_punti); yl=zeros(1,tot_punti); zl=zeros(1,tot_punti);

sf=0.13;

% primo tratto circolare
tt=[]; v=[]; a=[]; vx=[]; vy=[]; ax=[]; ay=[];
[R,xcyc]=fit_circle_through_3_points([0 3; -1 2; 0 1]);
xc=xcyc(1)*sf; yc=xcyc(2)*sf;
R=sf*R;
th=linspace(0,3*pi/2,approx1); 
xl1=R*cos(th)+xc; 
xl(1:approx1)=xl1;
yl1=R*sin(th)+yc;
yl(1:approx1)=yl1;

[tt1,v1,a1,vx1,vy1,ax1,ay1] = look_ahead(xl1,yl1,0,0,v_nodo,vmax*0.8,amax,approx1,xc,yc,R,1);
tt=[tt1]; v=[v1]; a=[a1]; vx=[vx1]; vy=[vy1]; ax=[ax1]; ay=[ay1];

 % secondo tratto circolare
[R,xcyc]=fit_circle_through_3_points([0 1; 1 0; 0 -1]);
xc=xcyc(1)*sf; yc=xcyc(2)*sf;
R=sf*R;
th=linspace(pi/2,-2*pi-pi/2,approx2);
xl2=R*cos(th)+xc;
xl(approx1:approx1+approx2-1)=xl2;
yl2=R*sin(th)+yc;
yl(approx1:approx1+approx2-1)=yl2;
[tt2,v2,a2,vx2,vy2,ax2,ay2] = look_ahead(xl2,yl2,tt(end),v(end),0,vmax*0.8,amax,approx2,xc,yc,R,2);
tt=[tt tt2(2:end)]; v=[v v2(2:end)] ; a=[a a2(2:end)]; vx=[vx vx2(2:end)]; vy=[vy vy2(2:end)]; ax=[ax ax2(2:end)]; ay=[ay ay2(2:end)];

 % terzo tratto circolare
[R,xcyc]=fit_circle_through_3_points([0 -1; 1 -2; 0 -3]);
xc=xcyc(1)*sf; yc=xcyc(2)*sf;
R=sf*R;
th=linspace(pi/2,-pi,approx3);
xl3=R*cos(th)+xc;
xl(approx1+approx2-1:approx1+approx2+approx3-2)=xl3;
yl3=R*sin(th)+yc;
yl(approx1+approx2-1:approx1+approx2+approx3-2)=yl3;
[tt3,v3,a3,vx3,vy3,ax3,ay3] = look_ahead(xl3,yl3,tt(end),v(end),0,vmax*0.8,amax,approx3,xc,yc,R,2);
tt_l=[tt tt3(2:end)]; v_l=[v v3(2:end)] ; a_l=[a a3(2:end)]; vx=[vx vx3(2:end)]; vy=[vy vy3(2:end)]; ax=[ax ax3(2:end)]; ay=[ay ay3(2:end)];

% rotazione in 3d
zl=zeros(1,length(xl));
vz=zeros(1,length(vx));
az=zeros(1,length(ax));
[x,y,z] = Rotation3D_MAAT(xl,yl,zl,pi/2.7,pi/8,-pi/6);
[vx,vy,vz] = Rotation3D_MAAT(vx,vy,vz,pi/2.7,pi/8,-pi/6);
[ax,ay,az] = Rotation3D_MAAT(ax,ay,az,pi/2.7,pi/8,-pi/6);

% traslazione
x=x+0.5; y=y+1.15; z=z+0.75;

% tratto iniziale
approxi=floor((((x(1)-0.3)^2 +(y(1)-(0.5+0.1+0.5))^2 +(z(1)-0.8)^2)^0.5)*1002);
qini=RPR_inv([0.3 1.1 0.8],Length);
qfin=RPR_inv([x(1) y(1) z(1)],Length);

[tt_ti,qi_1,qi_2,qi_3,qpi_1,qpi_2,qpi_3,qppi_1,qppi_2,qppi_3] = Traj_min_act_time(qini,qfin,qpmax,qppmax,approxi);

for i=1:approxi
    si=RPR_dir([qi_1(i); qi_2(i); qi_3(i)],Length);
    x_ti(i)=si(1);    y_ti(i)=si(2);    z_ti(i)=si(3);
    s_ti(i)=sqrt(x_ti(i)^2 +y_ti(i)^2 +z_ti(i)^2);
    Jf=RPR_jac([qi_1(i); qi_2(i); qi_3(i)],Length);
    vi=Jf*[qpi_1(i); qpi_2(i); qpi_3(i)];
    vx_ti(i)=vi(1);    vy_ti(i)=vi(2);    vz_ti(i)=vi(3);
    v_ti(i)=sqrt(vx_ti(i)^2 +vy_ti(i)^2 +vz_ti(i)^2);
    Jpf=RPR_jacp([qi_1(i); qi_2(i); qi_3(i)],[qpi_1(i); qpi_2(i); qpi_3(i)],Length);
    ai=Jpf*[qpi_1(i); qpi_2(i); qpi_3(i)]+Jf*[qppi_1(i); qppi_2(i); qppi_3(i)];
    ax_ti(i)=ai(1);    ay_ti(i)=ai(2);    az_ti(i)=ai(3);
    a_ti(i)=sqrt(ax_ti(i)^2 +ay_ti(i)^2 +az_ti(i)^2);
end

% tratto finale
approxf=floor((((x(end)-0.3)^2 +(y(end)-(0.5+0.1+0.5))^2 +(z(end)-0.8)^2)^0.5)*1002);
qini=RPR_inv([x(end) y(end) z(end)]',Length);
qfin=RPR_inv([0.3 1.1 0.8]',Length);

[tt_tf,qf_1,qf_2,qf_3,qpf_1,qpf_2,qpf_3,qppf_1,qppf_2,qppf_3] = Traj_min_act_time(qini,qfin,qpmax,qppmax,approxf);

for i=1:approxf
    sf=RPR_dir([qf_1(i); qf_2(i); qf_3(i)],Length);
    x_tf(i)=sf(1);    y_tf(i)=sf(2);    z_tf(i)=sf(3);
    s_tf(i)=sqrt(x_tf(i)^2 +y_tf(i)^2 +z_tf(i)^2);
    Jf=RPR_jac([qf_1(i); qf_2(i); qf_3(i)],Length);
    vf=Jf*[qpf_1(i); qpf_2(i); qpf_3(i)];
    vx_tf(i)=vf(1);    vy_tf(i)=vf(2);    vz_tf(i)=vf(3);
    v_tf(i)=sqrt(vx_tf(i)^2 +vy_tf(i)^2 +vz_tf(i)^2);
    Jpf=RPR_jacp([qf_1(i); qf_2(i); qf_3(i)],[qpf_1(i); qpf_2(i); qpf_3(i)],Length);
    af=Jpf*[qpf_1(i); qpf_2(i); qpf_3(i)]+Jf*[qppf_1(i); qppf_2(i); qppf_3(i)];
    ax_tf(i)=af(1);    ay_tf(i)=af(2);    az_tf(i)=af(3);
    a_tf(i)=sqrt(ax_tf(i)^2 +ay_tf(i)^2 +az_tf(i)^2);
end

tt=[tt_ti(1:end-1) tt_l(1:end-1)+tt_ti(end) tt_tf(1:end)+tt_ti(end)+tt_l(end)]; v=[v_ti(1:end-1) v_l(1:end-1) v_tf(1:end)]; a=[a_ti(1:end-1) a_l(1:end-1) a_tf(1:end)];
vx=[vx_ti(1:end-1) vx(1:end-1) vx_tf(1:end)]; vy=[vy_ti(1:end-1) vy(1:end-1) vy_tf(1:end)]; vz=[vz_ti(1:end-1) vz(1:end-1) vz_tf(1:end)];  ax=[ax_ti(1:end-1) ax(1:end-1) ax_tf(1:end)]; ay=[ay_ti(1:end-1) ay(1:end-1) ay_tf(1:end)]; az=[az_ti(1:end-1) az(1:end-1) az_tf(1:end)];
x=[x_ti(1:end-1) x(1:end-1) x_tf(1:end)]; y=[y_ti(1:end-1) y(1:end-1) y_tf(1:end)]; z=[z_ti(1:end-1) z(1:end-1) z_tf(1:end)];
dT=diff(tt);

vg=v; ag=a;

% Forces and torques external for gripper
fx=0; fy=0; fz=0;
cx=0; cy=0; cz=0;
ext=[0 -cz cy fx; cz 0 -cx fy; -cy cx 0 fz; -fx -fy -fz 0]; % external forces and torques matrix

% Data for inertia matrix
m1=27.1434; m2=12.96; m3=21.6; m4=10.56; m5=3.80365; mg=0; m=[m1,m2,m3,m4,m5,mg]; % masses
g1=l1/2; g2=l2/2; g3= l3/2; g4=l4/2; g5=l5/2; G=[g1,g2,g3,g4,g5]; % position of center of masses from distal 
jg1=0; jx1=jg1; jy1=jg1; jz1=jg1;
jg2=m2*(l2^2)/12; jx2=0; jy2=jg2; jz2=jg2; 
jg3=m3*(l3^2)/12; jx3=jg3; jy3=0; jz3=jg3;
jg4=m4*(l4^2)/12; jx4=jg4; jy4=0; jz4=jg4; 
jg5=m5*(l5^2)/12; jx5=jg5; jy5=0; jz5=jg5; 
jxy=0; jyz=0; jxz=0;
J1=inertia1(jx2,jy2,jz2,m2,l2,g2,jx3,jy3,jz3,m3,l3,g3); % joint 1 inertia matrix
J2=inertia(jx4,jy4,jz4,m4,l4,g4); % joint 2 inertia matrix
J3=inertia(jx5,jy5,jz5,m5,l5,g5); % joint 3 inertia matrix
J=zeros(4,4,3);
J(:,:,1)=J1; J(:,:,2)=J2; J(:,:,3)=J3; % total inertia matrix
Hg=[0 0 0 0;0 0 0 0;0 0 0 -9.81;0 0 0 0]; % gravity matrix
Mass_matrix=diag([mg,mg,mg,m5,m5,m5,m4,m4,m4,m3,m3,m3,m2,m2,m2,jg2,jg3+jg4,jg5]);
Fs=zeros(18,1);
Fs(1)=fx; Fs(2)=fy; Fs(3)=fz;
Ag=zeros(18,1);
Ag(3)=-9.81; Ag(6)=-9.81; Ag(9)=-9.81; Ag(12)=-9.81; Ag(15)=-9.81; 
Fse=Fs+Mass_matrix*Ag;

% Joints limits
q1max=deg2rad(170); q1min=-q1max;
q2min=0.1; q2max=0.55;
q3max=deg2rad(130); q3min=-q3max;

% Initial and poses of the trajectory in space coordinates
q0=[0,q2min,0]'; % home position
si_f=RPR_dir(q0,Length);
sT=[si_f(1),x,si_f(1); si_f(2),y,si_f(2); si_f(3),z,si_f(3)];
s=[sT(1,1),sT(2,1),sT(3,1)]';
qi=INVnewton(s,q0,Length,1e-6,100);
% q1=RPR_inv(s,Length); %
npose=length(sT(1,:));

% Others initializations
prevcfg1=0; prevcfg2=0;
axs=axes;

for i=1:length(tt)
    s=[x(i) y(i) z(i)]'; % spatial gripper position
    v=[vx(i) vy(i) vz(i)]'; % spatial gripper velocity
    a=[ax(i) ay(i) az(i)]'; % spatial gripper acceleration

    % Inverse Kinematic without 4*4 matrix
%         q=INVnewton(s,q0,Length,1e-6,100); % q components
    q=RPR_inv(s,Length); % q components
%     if q(1)>q1max || q(1)<q1min || q(2)>q2max || q(2)<q2min || q(3)>q3max || q(3)<q3min
%         disp('The trajectory excedes working space ranges!')
%         break
%     end
    q1(i)=q(1); q2(i)=q(2); q3(i)=q(3); % q component
    Jac=RPR_jac(q,Length); % jacobian
    qp=Jac^(-1)*v; % q angular velocities
    qp1(i)=qp(1); qp2(i)=qp(2); qp3(i)=qp(3); % q angular velocity components
    Jacp=RPR_jacp(q,qp,Length); % jacobian derivate
    qpp=Jac^(-1)*(a-Jacp*qp); % q angular accelerations
    qpp1(i)=qpp(1); qpp2(i)=qpp(2); qpp3(i)=qpp(3); % q angular acceleration components
%         q0=q; % uncomment only if using function INVnewton

    % Initialization of matrices
    L=zeros(4,4,nlink); % relative position matrix
    T=zeros(4,4,nlink+1); T(:,:,1)=diag(ones(4,1)); % relative and absolute position matrix
    M=zeros(4,4,nlink); M0=zeros(4,4,nlink); % joint L matrix in local and absolute frame
    W=zeros(4,4,nlink); W0=zeros(4,4,nlink); % relative velocity in local and absolute frame
    H=zeros(4,4,nlink); H0=zeros(4,4,nlink); % relative acceleration in local and absolute frame
    WA=zeros(4,4,nlink+1); HA=zeros(4,4,nlink+1); % absolute velocity and acceleration in local and absolute frame

    % Direct kinematic with 4*4 matrix
    for j=1:1:nlink
        L(:,:,j)=L_matrix(j);
        M(:,:,j)=M_matrix(q,Length,j); % relative position matrix
        W(:,:,j)=L(:,:,j)*qp(j); % relative velocity matrix
        H(:,:,j)=L(:,:,j)*qpp(j)+(L(:,:,j)^2)*(qp(j)^2); % relative acceleration matrix
        T(:,:,j+1)=T(:,:,j)*M(:,:,j); % absolute position matrix
        W0(:,:,j)=T(:,:,j)*W(:,:,j)*T(:,:,j)^(-1); % trasform relative velocity matrix from actual frame to base frame
        L0(:,:,j)=T(:,:,j)*L(:,:,j)*T(:,:,j)^(-1); % trasform relative joint type matrix from actual frame to base frame
        H0(:,:,j)=T(:,:,j)*H(:,:,j)*T(:,:,j)^(-1); % trasform relative acceleration matrix from actual frame to base frame
        WA(:,:,j+1)=WA(:,:,j)+W0(:,:,j); % absolute velocity matrix
        HA(:,:,j+1)=HA(:,:,j)+H0(:,:,j)+2*WA(:,:,j)*W0(:,:,j); % absolute acceleration matrix
    end
    P=T(:,4,nlink+1); Pp=WA(:,:,nlink+1)*P; Ppp=HA(:,:,nlink+1)*P; % gripper position, velocity, acceleration
    x4(i)=P(1); y4(i)=P(2); z4(i)=P(3); % x,y,z position
    vx4(i)=Pp(1); vy4(i)=Pp(2); vz4(i)=Pp(3); % x,y,z velocity
    ax4(i)=Ppp(1); ay4(i)=Ppp(2); az4(i)=Ppp(3); % x,y,z acceleration

    % Inverse dynamic with 4*4 matrix
    pow(i)=0;
    Ek(i)=0;
    Ep(i)=0;
    TA=diag(ones(4,1)); TA(:,4)=T(:,4,4); % transfer external forces from gripper to base frame
    ACT0(:,:,4)=TA*ext*TA'; % external force and torques on gripper to base frame
    for j=nlink:-1:1
        J0(:,:,j)=T(:,:,j+1)*J(:,:,j)*T(:,:,j+1)'; % inertia to absolute frame
        Ht=Hg-HA(:,:,j+1); % total acceleration
        FI(:,:,j)=Ht*J0(:,:,j)-J0(:,:,j)*Ht'; % weight+inertia forces on link
        ACT0(:,:,j)=FI(:,:,j)+ACT0(:,:,j+1); % forces between link i-1 and i
        tor(j,i)=-PseDot(ACT0(:,:,j),L0(:,:,j)); % joint torque
        Ek(i)=Ek(i)+trace(0.5*WA(:,:,j+1)*J0(:,:,j)*WA(:,:,j+1)'); % kinetic energy
        Ep(i)=Ep(i)-trace(Hg*J0(:,:,j)); % potential energy
        w(nlink-j+1,i)=-PseDot(ACT0(:,:,j),W0(:,:,j)); % joint power
        pow(i)=pow(i)-PseDot(ACT0(:,:,j),W0(:,:,j))+PseDot(ACT0(:,:,nlink+1),WA(:,:,j)); % absolute power for debug
    end
    w(nlink+1,i)=PseDot(ACT0(:,:,nlink+1),WA(:,:,nlink+1)); % absolute power to base frame
    w_tot(i)=w(1,i)+w(2,i)+w(3,i)+w(4,i); % total power

    % Dynamic without 4*4
    Se=RPR_dir_ext(q,Length,G); % extended positions
    Je=RPR_jac_ext(q,Length,G); % extended jacobian
    Jep=RPR_jacp_ext(q,qp,Length,G); % derivative extended jacobian
    Sep=Je*qp; % velocity extended 
    Fq=(Je'*Mass_matrix*Je)*qpp+Je'*Mass_matrix*Jep*qp-Je'*Fse; % joints torque
    Fq1(i)=Fq(1); Fq2(i)=Fq(2); Fq3(i)=Fq(3);
    Ek_ext(i)=0.5*Sep'*Mass_matrix*Sep; % kinetic energy
    Ep_ext(i)=-Ag'*Mass_matrix*Se; % potential energy
    Etot(i)=Ek_ext(i)+Ep_ext(i); % total energy
    W_ext(i)=Fq'*qp+Fs'*Sep; % power

    % Plot trajectory dynamically
    if rem(i,20)==0
        h1=figure(1);
        set(h1,'name','3D trajectory');
        grid on
        title('Manipulators trajectory motion');
        xlabel('x'); ylabel('y'); zlabel('z');
        axis equal
        [cfg1,cfg2,h1,h2,h3,h4,h5,h6]=plotRPR(q,Length,'b',1,axs);
        if prevcfg1~=0
            delete(prevcfg1);
            delete(prevh1);delete(prevh2);delete(prevh3);delete(prevh4);delete(prevh5);delete(prevh6);
        end
        prevcfg1=cfg1;
        prevh1=h1; prevh2=h2; prevh3=h3; prevh4=h4; prevh5=h5; prevh6=h6;
        if prevcfg2~=0
            delete(prevcfg2);
        end
        prevcfg2=cfg2;
        hold on
        plot3(P(1),P(2),P(3),'.g');
        grid on
        view(27,26); % point of view azimut, elevation
        hold on
    end
end
close(1)

%% Graphs

% plot 3D trajectory and poses
axs=axes;
h1=figure(1); % 3D plot of manipulator
set(h1,'name','3D representation xyz')
grid on
title('Manipulators poses and trajectory');
xlabel('x'); ylabel('y'); zlabel('z');
axis equal
plotRPR([q1(1,1);q2(1,1);q3(1,1)],Length,'b',1,axs);
plot3(x,y,z,'y','DisplayName','Trajectory','LineWidth',2); % trajectory
plot3(x4,y4,z4,'g','DisplayName','Trajectory 4*4','LineWidth',2); % trajectory
legend('Initial position','','Trajectory','Trajectory 4*4','Location','best');
view(27,26) % point of view azimut, elevation

h2=figure(2); % plot the position, velocity, acceleration of look-ahead algorithm
set(h2,'name','Motion of the gripper using look-ahead alogorithm');
plot(tt_l,v_l,tt_l,a_l,'LineWidth',1) % plot law of motion
ylim([-1.2,1.2])
legend('v','a','Location','best')
grid on
title('Motion of the gripper using look ahead algorithm')

h3=figure(3); % plot the position, velocity, acceleration of the y component of gripper
set(h3,'name','Motion on x - law cycloidal for q');
vxd=diff(x)./dT; axd=diff(vx)./dT; % debug
vxd(4218)=0; axd(4218)=0;
plot(tt,x,'--',tt,vx,'--',tt,ax,'--',tt,x4,tt,vx4,tt,ax4,tt(1:end-1),vxd,':',tt(1:end-1),axd,':') % plot law of motion
legend('s','v','a','s4*4','v4*4','a4*4','#v','#a','Location','best')
grid on
title('motion on x')

h4=figure(4); % plot the position, velocity, acceleration of the z component of gripper
set(h4,'name','Motion on y - law cycloidal for q')
vyd=diff(y)./dT; ayd=diff(vy)./dT; % debug
vyd(4218)=0; ayd(4218)=0;
plot(tt,y,'--',tt,vy,'--',tt,ay,'--',tt,y4,tt,vy4,tt,ay4,tt(1:end-1),vyd,':',tt(1:end-1),ayd,':') % plot law of motion
legend('s','v','a','s4*4','v4*4','a4*4','#v','#a','Location','best')
grid on
title('motion on y')

h5=figure(5); % plot the position, velocity, acceleration of the z component of gripper
set(h5,'name','Motion on z - law cycloidal for q')
vzd=diff(z)./dT; azd=diff(vz)./dT; % debug
vzd(4218)=0; azd(4218)=0;
plot(tt,z,'--',tt,vz,'--',tt,az,'--',tt,z4,tt,vz4,tt,az4,tt(1:end-1),vzd,':',tt(1:end-1),azd,':') % plot law of motion
legend('s','v','a','s4*4','v4*4','a4*4','#v','#a','Location','best')
grid on
title('motion on z')

h6=figure(6); % plot the angular position, velocity, acceleration of rotational joint 1
set(h6,'name','Joint 1 - law cycloidal for q');
plot(tt,q1,tt,qp1,tt,qpp1) % plot law of motion
legend('q','qp','qpp','Location','best')
grid on
title('Rotational joint 1')

h7=figure(7); % plot the angular position, velocity, acceleration of prismatic joint 2
set(h7,'name','Joint 2 - law cycloidal for q')
plot(tt,q2,tt,qp2,tt,qpp2) % plot law of motion
legend('q','qp','qpp','Location','best')
grid on
title('Prismatic joint 2')

h8=figure(8); % plot the angular position, velocity, acceleration of rotational joint 3
set(h8,'name','Joint 3 - law cycloidal for q')
plot(tt,q3,tt,qp3,tt,qpp3) % plot law of motion
legend('q','qp','qpp','Location','best')
grid on
title('Rotational joint 3')

h9=figure(9); % plot the torques of each joint
set(h9,'name','Motor torques')
xlabel('Tempo [s]')
ylabel('Coppia [N*m]')
subplot(3,1,1); plot(tt,tor(1,:),tt,Fq1); title('Motor 1'); grid on; legend('torque 4*4','torque Jext','Location','best') % torque motor 1 
xlabel('Tempo [s]')
ylabel('Coppia [N*m]')
subplot(3,1,2); plot(tt,tor(2,:),tt,Fq2); title('Motor 2'); grid on; legend('torque 4*4','torque Jext','Location','best') % torque motor 2
xlabel('Tempo [s]')
ylabel('Forza [N]')
subplot(3,1,3); plot(tt,tor(3,:),tt,Fq3); title('Motor 3'); grid on; legend('torque 4*4','torque Jext','Location','best') % torque motor 3
xlabel('Tempo [s]')
ylabel('Coppia [N*m]')

h10=figure(10); % plot the Potential and Kinetic Energy
set(h10,'name','Potential and Kinetic Energy')

subplot(2,1,1); plot(tt,Ep, tt,Ep_ext); title('Potential Energy'); grid on; ylim([375 390]); legend('Ep 4*4','Ep Jext','Location','best') % Potential Energy
xlabel('Tempo [s]')
ylabel('Energia [J]')
subplot(2,1,2); plot(tt,Ek, tt,Ek_ext); title('Kinetic Energy'); grid on; ylim([-0.1 1.7]); legend('Ek 4*4','Ek Jext','Location','best') % Kinetic Energy
xlabel('Tempo [s]')
ylabel('Energia [J]')

h11=figure(11); % debug of power torques
pow_debug=diff(Ep+Ek)./dT;
pow_debug(4218)=0;
set(h11,'name','Debug of Power')
plot(tt,w_tot,'b',tt(1:end-1),pow_debug,'r', tt,W_ext);
grid on
xlabel('Tempo [s]')
ylabel('Potenza [W]')
legend('Power Fq+Fs 4*4','dE/dT','Power Jext','Location','best')
title('dEnergy/dt and Power of Forces (motors+external)')

h12=figure(12); % plot of power of joints 1, 2 ,3, ext_f, total
set(h12,'name','Power of Joints')
plot(tt,w(1,:),tt,w(2,:),tt,w(3,:),tt,w(4,:),tt,w_tot)
xlabel('Tempo [s]')
ylabel('Potenza [W]')
legend('w1','w2','w3','wf','wtot','Location','best')
grid on
title('Power of Joints')

%% Working space

% working space on xy plane
h13=figure(13);
set(h13,'name','Working space plane on xy')
axs=axes;
title('Working space plane on xy')
plotRPR([q1max;q2max;0],Length,'r',13,axs);
hold on
plotRPR([q1min;q2min;q3min],Length,'r',13,axs);
hold on
plotRPR([0;q2min;0],Length,'b',13,axs);
d1=sqrt(l2^2+(l3+l5*cos(q3max)+q2min)^2);
d2=sqrt(l2^2+(l3+q2max+l5)^2);
i=1;
hold on
for d=[d1,d2]
    xp=linspace(-d,d,1000);
    x2=linspace(-d,d*cos(q1max+acos(l2/d)),1000);
    x3=linspace(d*cos(q1min+acos(l2/d)),d,1000);
    yp=sqrt(d^2-xp.^2);
    y2=-sqrt(d^2-x2.^2);
    y3=-sqrt(d^2-x3.^2);
    plot(xp,yp,'g','LineWidth',2)
    hold on
    plot(x2,y2,'g','LineWidth',2)
    plot(x3,y3,'g','LineWidth',2)
    grid on
    X2(i)=x2(end);
    Y2(i)=y2(end);
    X3(i)=x3(1);
    Y3(i)=y3(1);
    plot(X2,Y2,'g','LineWidth',2)
    plot(X3,Y3,'g','LineWidth',2)
    axis equal
    i=i+1;
end
for d=d1:0.05:d2
    xp=linspace(-d,d,1000);
    x2=linspace(-d,d*cos(q1max+acos(l2/d)),1000);
    x3=linspace(d*cos(q1min+acos(l2/d)),d,1000);
    yp=sqrt(d^2-xp.^2);
    y2=-sqrt(d^2-x2.^2);
    y3=-sqrt(d^2-x3.^2);
    plot(xp,yp,'g')
    hold on
    plot(x2,y2,'g')
    plot(x3,y3,'g')
end
legend('Limit positions','','','','Initial position','','','Working space limit')
hold off
clear axs

% working space on yz plane
h14=figure(14);
set(h14,'name','Working space plane on yz')
axs=axes;
title('Working space on plane yz')
hold on
plotRPR([0;q2min;q3max],Length,'r',14,axs);
hold on
plotRPR([0;q2max;q3max],Length,'r',14,axs);
hold on
plotRPR([0;q2min;0],Length,'b',14,axs);
hold on
zp=linspace(l3+q2min,l3+q2max);
z1_1=linspace(l1+l5,l1+l5);
z1_2=linspace(l1-l5,l1-l5);
plot3(zeros(length(zp)),zp,z1_1,'g','LineWidth',2);
hold on
plot3(zeros(length(zp)),zp,z1_2,'g','LineWidth',2);
hold on
z2=linspace(l3+q2min+l5*cos(q3max),l3+q2max+l5*cos(q3max));
z2_1=linspace(l1+l5*sin(q3max),l1+l5*sin(q3max));
z2_2=linspace(l1-l5*sin(q3max),l1-l5*sin(q3max));
plot3(zeros(length(z2_2)),z2,z2_1,'g','LineWidth',2);
hold on
plot3(zeros(length(z2)),z2,z2_2,'g','LineWidth',2)
hold on
t3=linspace(q3min,q3max);
y3 = l5*cos(t3)+l3+q2max;
z3 = l5*sin(t3)+l1;
plot3(zeros(length(z3)),y3,z3,'g','LineWidth',2)
hold on
t4=linspace(q3min+(q3max-pi/2),q3max-(q3max-pi/2));
y4 = l5*cos(t4)+l3+q2min;
z4 = l5*sin(t4)+l1;
plot3(zeros(length(z4)),y4,z4,'g','LineWidth',2)
hold on
t5=linspace(q3min,-pi/2);
y5 = l5*cos(t5)+l3+q2min;
z5 = l5*sin(t5)+l1;
plot3(zeros(length(z5)),y5,z5,'g','LineWidth',2)
hold on
t6=linspace(q3max,pi/2);
y6 = l5*cos(t6)+l3+q2min;
z6 = l5*sin(t6)+l1;
plot3(zeros(length(z6)),y6,z6,'g','LineWidth',2)
hold on
for qqi=q2min:0.03:q2max
    t7=linspace(q3max,q3min);
    y7 = l5*cos(t7)+l3+qqi;
    z7 = l5*sin(t7)+l1;
    plot3(zeros(length(z7)),y7,z7,'g')
    hold on
end
view(90,0);
ylim([-0.1,1.6])
zlim([0,1.6])
legend('Limit positions','','','','Initial position','','','','Working space limit','','Location','southwest')
hold off
clear axs

% 3D working space + trajectory 
h15=figure(15);
set(h15,'name','3D Working space and Trajectory')
axs=axes;
title('3D Working space and Trajectory')
alfa=q1min:0.1:q1max;
beta=[q2min,q2max];
gamma=q3min:0.1:q3max;
plot3(x,y,z,'r','LineWidth',2)
hold on
plotRPR([0;0.1;0],Length,'b',15,axs);
hold on
for i=1:length(alfa)
    for j=1:length(beta)
        h=1;
        for k=1:length(gamma)
            x1(h)=l2*cos(alfa(i))-(l3+beta(j)+l5*cos(gamma(k)))*sin(alfa(i));
            y1(h)=l2*sin(alfa(i))+(l3+beta(j)+l5*cos(gamma(k)))*cos(alfa(i));
            z1(h)=l1+l5*sin(gamma(k));
            h=h+1;
        end
        plot3(x1(1:end),y1(1:end),z1(1:end),'-g','LineWidth',0.2)
        axis equal
        hold on
    end
end
h=1;
for i=1:length(alfa)
    x1(h)=l2*cos(alfa(i))-(l3+beta(1)+l5*cos(gamma(1)))*sin(alfa(i));
    x1(h+1)=l2*cos(alfa(i))-(l3+beta(end)+l5*cos(gamma(1)))*sin(alfa(i));
    y1(h)=l2*sin(alfa(i))+(l3+beta(1)+l5*cos(gamma(1)))*cos(alfa(i));
    y1(h+1)=l2*sin(alfa(i))+(l3+beta(end)+l5*cos(gamma(1)))*cos(alfa(i));
    z1(h)=l1+l5*sin(gamma(1));
    z1(h+1)=l1+l5*sin(gamma(1));
    h=h+2;
end
plot3(x1(1:end),y1(1:end),z1(1:end),'-g','LineWidth',0.2)
axis equal
hold on
h=1;
for i=1:length(alfa)
    x1(h)=l2*cos(alfa(i))-(l3+beta(1)+l5*cos(gamma(end)))*sin(alfa(i));
    x1(h+1)=l2*cos(alfa(i))-(l3+beta(end)+l5*cos(gamma(end)))*sin(alfa(i));
    y1(h)=l2*sin(alfa(i))+(l3+beta(1)+l5*cos(gamma(end)))*cos(alfa(i));
    y1(h+1)=l2*sin(alfa(i))+(l3+beta(end)+l5*cos(gamma(end)))*cos(alfa(i));
    z1(h)=l1+l5*sin(gamma(end));
    z1(h+1)=l1+l5*sin(gamma(end));
    h=h+2;
end
axis equal
plot3(x1(1:end),y1(1:end),z1(1:end),'-g','LineWidth',0.2)
legend('Trajectory','Initial position','','','Working space','Location','best')
