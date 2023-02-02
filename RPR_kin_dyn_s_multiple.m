% Project Robot RPR with trajectory imposed on s

clear all
close all
clc

% Link lengths
l1=0.84; % link 1
l2=0.3; % link 2
l3=0.4; % link 3
l4=0.35;
l5=0.35;
Length=[l1, l2, l3, l4, l5]';
nlink=3; % number of mobile links of the robot

% Initial and poses of the trajectory in space coordinates
sT=[0.4, -0.7, -0.6; % x
    0.8, 0.9, 0.5; % y
    0.7, 1, 0.5]; % z
s=[sT(1,1),sT(2,1),sT(3,1)]';
q0=[0,0,0]'; % starting point for inverse kinematic
qi=INVnewton(s,q0,Length,1e-6,100);
npose=length(sT(1,:));

% Trajectory
Time=5; % motion time
n=100; % n. of points of the law
dT=Time/(n-1); % time step

% Forces and torques external for gripper
fx=1; fy=1; fz=1;
cx=0; cy=0; cz=0;
ext=[0 -cz cy fx; cz 0 -cx fy; -cy cx 0 fz; -fx -fy -fz 0]; % external forces and torques matrix

% data for inertia matrix
m1=8; m2=7; m3=4; m4=4; m5=5; m=[m1,m2,m3,m4,m5]; % masses
g1=0.48; g2=0.42; g3= 0.24; g4=0.4; g5=0.5; g=[g1,g2,g3,g4,g5]; % position of center of masses from distal 
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

% Others initializations
prevcfg1=0; prevcfg2=0;
axs=axes;

for k=1:1:npose-1
    % Graph joint coordinate
    xi = s(1); % initial value
    xf = sT(1,k+1); % final value
    yi = s(2); % initial value
    yf = sT(2,k+1); % final value
    zi = s(3); % initial value
    zf = sT(3,k+1); % final value
    dx = xf-xi; % total motion
    dy = yf-yi; % total motion
    dz = zf-zi; % total motion

    for i=(k-1)*n+1:1:(k-1)*n+n
        t=(i-(k-1)*n-1)*Time/(n-1); % time from 0 to T with step dT
        tt(i)=t+(k-1)*Time;
        [x(i),vx(i),ax(i)]=cicloidale(t,Time,xi,dx); % x component
        [y(i),vy(i),ay(i)]=cicloidale(t,Time,yi,dy); % y component
        [z(i),vz(i),az(i)]=cicloidale(t,Time,zi,dz); % y component
        s=[x(i) y(i) z(i)]'; % spatial gripper position
        v=[vx(i) vy(i) vz(i)]'; % spatial gripper velocity
        a=[ax(i) ay(i) az(i)]'; % spatial gripper acceleration

        % Inverse Kinematic without 4*4 matrix
        q=INVnewton(s,q0,Length,1e-6,100); % q components
        q1(i)=q(1); q2(i)=q(2); q3(i)=q(3); % q component
        Jac=RPR_jac(q,Length); % jacobian
        qp=Jac^(-1)*v; % q angular velocities
        qp1(i)=qp(1); qp2(i)=qp(2); qp3(i)=qp(3); % q angular velocity components
        Jacp=RPR_jacp(q,qp,Length); % jacobian derivate
        qpp=Jac^(-1)*(a-Jacp*qp); % q angular accelerations
        qpp1(i)=qpp(1); qpp2(i)=qpp(2); qpp3(i)=qpp(3); % q angular acceleration components
        q0=q;

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

        % Plot trajectory dynamically
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
%%

% Plot 3D trajectory and poses
Plot_3D(sT,Length,npose,n,q1,q2,q3,x,y,z,x4,y4,z4,1)

% Plot motion on x,y,z axis
Plot_Motion_Axis(tt,dT,x,vx,ax,x4,vx4,ax4,y,vy,ay,y4,vy4,ay4,z,vz,az,z4,vz4,az4,2,3,4)

% Plot joints kinematic and torque
Plot_Joints(tt,q1,qp1,qpp1,q2,qp2,qpp2,q3,qp3,qpp3,tor,5,6,7,8)

% Plot kinematic energy, potential energy and power
Plot_Energy_Power(tt,Ep,Ek,w,w_tot,dT,9,10,11)