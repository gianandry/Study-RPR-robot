% Project Robot RPR with trajectory imposed on s

clear all
close all
clc

% others initializations
prevcfg1=0; prevcfg2=0;
axs=axes;

% link lenghts
l1=0.84; % link 1
l2=0.3; % link 2
l3=0.4; % link 3
l4=0.35;
l5=0.35;

Lenght=[l1, l2, l3, l4, l5]';
nlink=3; % number of links of the robot

% initial and final trajetory joints coordinates
si=[0.4,0.8,0.7]'; % initial spatial position
sf=[-0.7,0.9,1]'; % final spatial position
q0=[0,0,0]'; % starting point for inverse kinematic
qi=INVnewton(si,q0,Lenght,1e-6,100);

% trajectory
Time=2; % motion time
n=100; % n. of points of the law
dT=Time/(n-1); % time step

% graph joint coordinate
xi = si(1); % initial value
xf = sf(1); % final value
yi = si(2); % initial value
yf = sf(2); % final value
zi = si(3); % initial value
zf = sf(3); % final value
dx = xf-xi; % total motion
dy = yf-yi; % total motion 
dz = zf-zi; % total motion

% forces and torques external for gripper
fx=1; fy=1; fz=1;
cx=0; cy=0; cz=0;
ext=[0 -cz cy fx; cz 0 -cx fy; -cy cx 0 fz; -fx -fy -fz 0]; % external forces and torques matrix

% data for inertia matrix
m1=8; m2=7; m3=4; m4=4; m5=5; m=[m1,m2,m3,m4,m5]; % masses
g1=l1/2; g2=l2/2; g3= l3/2; g4=l4/2; g5=l5/2; g=[g1,g2,g3,g4,g5]; % position of center of masses from distal 
jg1=0.427; jy1=jg1; jz1=jg1; 
jg2=0.286; jy2=jg2; jz2=jg2; 
jg3=0.0533; jy3=jg3; jz3=jg3;
jg4=0.286; jy4=jg4; jz4=jg4; 
jg5=0.0533; jy5=jg5; jz5=jg5; 
jx1=0; jx2=0; jx3=0; jx4=0; jx5=0;
jxy=0; jyz=0; jxz=0;
J1=inertia1(jx2,jy2,jz2,m2,l2,g2,jx3,jy3,jz3,m3,l3,g3); % joint 1 inertia matrix
J2=inertia(jx4,jy4,jz4,m4,l4,g4); % joint 2 inertia matrix
J3=inertia(jx5,jy5,jz5,m5,l5,g5); % joint 3 inertia matrix
J(:,:,1)=J1; J(:,:,2)=J2; J(:,:,3)=J3; % total inertia matrix
Hg=[0 0 0 0;0 0 0 0;0 0 0 -9.81;0 0 0 0]; % gravity matrix

for i=1:1:n
    t=(i-1)*Time/(n-1); % time from 0 to T with step dT
    tt(i)=t;
    [x(i),vx(i),ax(i)]=cicloidale(t,Time,xi,dx); % x component 
    [y(i),vy(i),ay(i)]=cicloidale(t,Time,yi,dy); % y component
    [z(i),vz(i),az(i)]=cicloidale(t,Time,zi,dz); % y component
    s=[x(i) y(i) z(i)]'; % spatial gripper position
    v=[vx(i) vy(i) vz(i)]'; % spatial gripper velocity
    a=[ax(i) ay(i) az(i)]'; % spatial gripper acceleration

    % inverse kinematic without 4*4 matrix
%      q=INVnewton(s,q0,Lenght,1e-6,100); % q components
    q=RPR_inv(s,Lenght); % q components
    q1(i)=q(1); q2(i)=q(2); q3(i)=q(3); % q component
    Jac=RPR_jac(q,Lenght); % jacobian
    qp=Jac^(-1)*v; % q angular velocities
    qp1(i)=qp(1); qp2(i)=qp(2); qp3(i)=qp(3); % q angular velocity components
    Jacp=RPR_jacp(q,qp,Lenght); % jacobian derivate
    qpp=Jac^(-1)*(a-Jacp*qp); % q angular accelerations
    qpp1(i)=qpp(1); qpp2(i)=qpp(2); qpp3(i)=qpp(3); % q angular acceleration components
    q0=q;

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
        M(:,:,j)=M_matrix(q,Lenght,j); % relative position matrix
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

    % plot trajectory dynamically
    h1=figure(1);
    set(h1,'name','3D trajectory');
    grid on
    title('Manipulators trajectory motion');
    xlabel('x'); ylabel('y'); zlabel('z');
    axis equal
    [cfg1,cfg2,h1,h2,h3,h4,h5,h6]=plotRPR(q,Lenght,'b',1,axs);   % Robot in pos. iniz.
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

end
close(1)

% plot 3D
axs=axes;
h1=figure(1); % 3D plot of manipulator
set(h1,'name','3D representation xyz')
grid on
title('Manipulators poses and trajectory');
xlabel('x'); ylabel('y'); zlabel('z');
axis equal
plotRPR(qi,Lenght,'r',1,axs); % Robot in pos. iniz.
plotRPR(q,Lenght,'b',1,axs); % Robot in pos. fin.
plot3(x,y,z,'y'); % trajectory
plot3(x4,y4,z4,'g'); % trajectory
legend('Initial pose','','','Final pose','','','Trajectory','Trajectory 4*4','Location','best');
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
subplot(3,1,1); plot(tt,tor(1,:)); ylabel('Motor 1'); grid on % torque motor 1
subplot(3,1,2); plot(tt,tor(2,:)); ylabel('Motor 2'); grid on % torque motor 2
subplot(3,1,3); plot(tt,tor(3,:)); ylabel('Motor 3'); grid on % torque motor 3

h9=figure(9); % plot the Potential and Kinetic Energy
set(h9,'name','Potential and Kinetic Energy')
subplot(2,1,1); plot(tt,Ep); ylabel('Potential Energy'); grid on % Potential Energy
subplot(2,1,2); plot(tt,Ek); ylabel('Kinetic Energy'); grid on % Kinetic Energy

h10=figure(10); % debug of power torques
pow_debug=diff(Ep+Ek)./dT;
set(h10,'name','Debug of Power')
plot(tt,w_tot,'b',tt(1:end-1),pow_debug,'r');
grid on
legend('Power Fq+Fs','dE/dT','Location','best')
title('dEnergy/dt and Power of Forces (motors+external)')

h11=figure(11); % plot of power of joints 1, 2 ,3, ext_f, total
set(h11,'name','Power of Joints')
plot(tt,w(1,:),tt,w(2,:),tt,w(3,:),tt,w(4,:),tt,w_tot)
legend('w1','w2','w3','wf','wtot','Location','best')
grid on
title('Power of Joints')