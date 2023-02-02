% Project Robot RPR with trajectory imposed on q

clear all
close all
clc

% others initializations
prevcfg1=0;
prevcfg2=0;
axs=axes;

% link lenghts
l1=0.8; % link 1
l2=0.3; % link 2
l3=0.5; % link 3
l4=0.6; % link 4
l5=0.5; % link 5
Length=[l1, l2, l3, l4, l5]';
nlink=3; % number of mobile links of the robot

% initial and final trajectory joints coordinates
qi=[0,0,pi/6]'; % initial joints position
qf=[pi/2,0.3,pi/2]'; % final joints position

% trajectory
Time=2; % motion time
n=100; % n. of points of the law
dT=Time/(n-1); % time step

% graph joint coordinate
q1i=qi(1); % initial q1 angle
q1f=qf(1); %final q1 angle
q2i=qi(2); % initial q2 angle
q2f=qf(2); %final q2 angle
q3i=qi(3); % initial q2 angle
q3f=qf(3); %final q2 angle
dq1=q1f-q1i;
dq2=q2f-q2i;
dq3=q3f-q3i;

% forces and torques external for gripper
fx=1; fy=1; fz=1;
cx=0; cy=0; cz=0;
ext=[0 -cz cy fx; cz 0 -cx fy; -cy cx 0 fz; -fx -fy -fz 0]; % external forces and torques matrix

% Data for inertia matrix
m1=36.2; m2=17.3; m3=28.8; m4=14.1; m5=5.1; mg=0; m=[m1,m2,m3,m4,m5,mg]; % masses
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

for i=1:1:n
    t=(i-1)*Time/(n-1); % time from 0 to T with step dT
    tt(i)=t;
    [q1(i),qp1(i),qpp1(i)]=cicloidale(t,Time,q1i,dq1);  % q1 components
    [q2(i),qp2(i),qpp2(i)]=cicloidale(t,Time,q2i,dq2);  % q2 components
    [q3(i),qp3(i),qpp3(i)]=cicloidale(t,Time,q3i,dq3);  % q3 components
    q=[q1(i) q2(i) q3(i)]'; % joints position
    qp=[qp1(i) qp2(i) qp3(i)]'; % joints velocity
    qpp=[qpp1(i) qpp2(i) qpp3(i)]'; % joint acceleration

    % direct kinematic without 4*4 matrix
    s=RPR_dir([q1(i),q2(i),q3(i)],Length);
    x(i)=s(1); % x position gripper
    y(i)=s(2); % y position gripper
    z(i)=s(3); % z position gripper
    Jac=RPR_jac(q,Length);
    v=Jac*qp; % velocities of gripper
    vx(i)=v(1); % x velocity gripper
    vy(i)=v(2); % y velocity gripper
    vz(i)=v(3); % z velocity gripper
    Jacp=RPR_jacp(q,qp,Length); % derivative of jacobian
    a=Jacp*qp+Jac*qpp; % accelerations of gripper
    ax(i)=a(1); % x acceleration
    ay(i)=a(2); % y acceleration
    az(i)=a(3); % y acceleration

    % initialization of matrices
    L=zeros(4,4,nlink); % relative position matrix
    T=zeros(4,4,nlink+1); T(:,:,1)=diag(ones(4,1)); % rel and abs position matrix
    M=zeros(4,4,nlink); M0=zeros(4,4,nlink); % joint L matrix in local and abs frame
    W=zeros(4,4,nlink); W0=zeros(4,4,nlink); % rel vel in local and abs frame
    H=zeros(4,4,nlink); H0=zeros(4,4,nlink); % rel acc in local and abs frame
    WA=zeros(4,4,nlink+1); HA=zeros(4,4,nlink+1); % abs vel and acc in local and abs frame

    % direct kinematic with 4*4 matrix
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
    xp4(i)=Pp(1); yp4(i)=Pp(2); zp4(i)=Pp(3); % x,y,z velocity
    xpp4(i)=Ppp(1); ypp4(i)=Ppp(2); zpp4(i)=Ppp(3); % x,y,z acceleration

    % inverse dynamic with 4*4 matrix
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

    % Dynamic without 4x4
    Se=RPR_dir_ext(q,Length,G); % extended positions
    Je=RPR_jac_ext(q,Length,G); % extended jacobian
    Jep=RPR_jacp_ext(q,qp,Length,G); % derivative extended jacobian
    Sep=Je*qp; % velocity extended 
    Fq=(Je'*Mass_matrix*Je)*qpp+Je'*Mass_matrix*Jep*qp-Je'*Fse; % joints torque
    Fq1(i)=Fq(1);
    Fq2(i)=Fq(2);
    Fq3(i)=Fq(3);
    Ek_ext(i)=0.5*Sep'*Mass_matrix*Sep; % kinetic energy
    Ep_ext(i)=-Ag'*Mass_matrix*Se; % potential energy
    Etot(i)=Ek_ext(i)+Ep_ext(i); % total energy
    W_ext(i)=Fq'*qp+Fs'*Sep; % power

    % plot trajectory dynamically
    h1=figure(1);
    set(h1,'name','3D trajectory');
    grid on
    title('Manipulators trajectory motion');
    xlabel('x'); ylabel('y'); zlabel('z');
    axis equal
    [cfg1,cfg2,h1,h2,h3,h4,h5,h6]=plotRPR(q,Length,'b',1,axs);   % Robot in pos. iniz.
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
    view(27,26); %point of view azimut, elevation
    hold on

end
close(1)
%% 

% plot 3D
axs=axes;
h1=figure(1); % 3D plot of manipulator
set(h1,'name','3D representation xyz')
grid on
title('Manipulators poses and trajectory');
xlabel('x'); ylabel('y'); zlabel('z');
axis equal
plotRPR(qi,Length,'r',1,axs); % Robot in pos. iniz.
plotRPR(qf,Length,'b',1,axs); % Robot in pos. fin.
plot3(x,y,z,'y'); % trajectory
plot3(x4,y4,z4,'g'); % trajectory
legend('Initial pose','','Final pose','','Trajectory','Trajectory 4*4','Location','best');
view(27,26) %point of view azimut, elevation

% other graphics 
h2=figure(2); % plot the position, velocity, acceleration of the x component of gripper
set(h2,'name','Motion on x - law cycloidal for q');
vxd=diff(x)/dT; axd=diff(vx)/dT; % debug
plot(tt,x,'--',tt,vx,'--',tt,ax,'--',tt,x4,tt,xp4,tt,xpp4,tt(1:end-1),vxd,':',tt(1:end-1),axd,':') % plot law of motion
legend('s','v','a','s4*4','v4*4','a4*4','#v','#a','Location','best')
grid on
title('motion on x')

h3=figure(3); % plot the position, velocity, acceleration of the y component of gripper
set(h3,'name','Motion on y - law cycloidal for q')
vyd=diff(y)/dT; ayd=diff(vy)/dT; % debug
plot(tt,y,'--',tt,vy,'--',tt,ay,'--',tt,y4,tt,yp4,tt,ypp4,tt(1:end-1),vyd,':',tt(1:end-1),ayd,':') % plot law of motion
legend('s','v','a','s4*4','v4*4','a4*4','#v','#a','Location','best')
grid on
title('motion on y')

h4=figure(4); % plot the position, velocity, acceleration of the z component of gripper
set(h4,'name','Motion on z - law cycloidal for q')
vzd=diff(z)/dT; azd=diff(vz)/dT; % debug
plot(tt,z,'--',tt,vz,'--',tt,az,'--',tt,z4,tt,zp4,tt,zpp4,tt(1:end-1),vzd,':',tt(1:end-1),azd,':') % plot law of motion
legend('s','v','a','s4*4','v4*4','a4*4','#v','#a','Location','best')
grid on
title('motion on z')

h5=figure(5); % plot the angular position, velocity, acceleration of rotational joint 1
set(h5,'name','Joint 1 - law cycloidal for q');
plot(tt,q1,tt,qp1,tt,qpp1) % plot law of motion
legend('q','qp','qpp','Location','best')
grid on
title('Rotational joint 1')

h6=figure(6); % plot the angular position, velocity, acceleration of prismatic joint 2
set(h6,'name','Joint 2 - law cycloidal for q')
plot(tt,q2,tt,qp2,tt,qpp2) % plot law of motion
legend('q','qp','qpp','Location','best')
grid on
title('Prismatic joint 2')

h7=figure(7); % plot the angular position, velocity, acceleration of rotational joint 3
set(h7,'name','Joint 3 - law cycloidal for q')
plot(tt,q3,tt,qp3,tt,qpp3) % plot law of motion
legend('q','qp','qpp','Location','best')
grid on
title('Rotational joint 3')

h8=figure(8); % plot the torques of each joint
set(h8,'name','Motor torques')
subplot(3,1,1); plot(tt,tor(1,:),tt,Fq1); ylabel('Motor 1'); grid on; legend('torque 4x4','torque Jext') % torque motor 1 
subplot(3,1,2); plot(tt,tor(2,:),tt,Fq2); ylabel('Motor 2'); grid on; legend('torque 4x4','torque Jext') % torque motor 2
subplot(3,1,3); plot(tt,tor(3,:),tt,Fq3); ylabel('Motor 3'); grid on; legend('torque 4x4','torque Jext') % torque motor 3

h9=figure(9); % plot the Potential and Kinetic Energy
set(h9,'name','Potential and Kinetic Energy')
subplot(2,1,1); plot(tt,Ep, tt,Ep_ext); ylabel('Potential Energy'); grid on; legend('Ep 4x4','Ep Jext') % Potential Energy
subplot(2,1,2); plot(tt,Ek, tt,Ek_ext); ylabel('Kinetic Energy'); grid on; legend('Ek 4x4','Ek Jext') % Kinetic Energy

h10=figure(10); % debug of power torques
pow_debug=diff(Ep+Ek)./dT;
set(h10,'name','Debug of Power')
plot(tt,w_tot,'b',tt(1:end-1),pow_debug,'r', tt,W_ext);
grid on
legend('Power Fq+Fs 4x4','dE/dT','Power Jext','Location','best')
title('dEnergy/dt and Power of Forces (motors+external)')

h11=figure(11); % plot of power of joints 1, 2 ,3, ext_f, total
set(h11,'name','Power of Joints')
plot(tt,w(1,:),tt,w(2,:),tt,w(3,:),tt,w(4,:),tt,w_tot)
legend('w1','w2','w3','wf','wtot','Location','best')
grid on
title('Power of Joints')