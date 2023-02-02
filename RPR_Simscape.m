% geometrical parameters for simscape robot simulation
density=3000;
% base
lb=0.1;
rb=0.12;
colorb=[0.5 0.5 0.5];
% link 1
l1=0.8;
r1=0.06;
color1=[1 0 0];
% link 2
l2=0.3;
w2=0.12;
h2=0.12;
r2=0.06;
color2=[1 0 0];
% link 3
l3=0.5;
w3=0.12;    
h3=0.12;
r3=0.06;
color3=[1 0 0];
% link 4
l4=0.65; % sarebbe 0.6 nel matlab 
w4=0.08;
v4=0.02;
o4=0.2;
h4=0.08;
f4=[0,w4/2; 0, -w4/2; l4, -w4/2; l4, -w4/2+v4; l4-o4, -w4/2+v4; l4-o4, w4/2-v4; l4, w4/2-v4; l4, w4/2];
color4=[0 1 0];
% link 5
l5=0.5;
w=0.04;
w5=0.1;
h5=0.06;
r5=0.05;
color5=[0 0 1];
A = linspace(-pi/2, pi/2, 5)';
B = linspace(pi/2, 3*pi/2, 5)';
csRight = [l5/2 + w/2*cos(A) w/2*sin(A)];
csLeft = [-l5/2 + w/2*cos(B) w/2*sin(B)];
cs5=[ csLeft; csRight];

%joint limits
q1min=deg2rad(-170); q1max=deg2rad(170);
q2min=0.1; q2max=0.55;
q3min=deg2rad(-130); q3max=deg2rad(130);

% 1 rotational joint
KpPos1=50;
TiPos1=1/50;
TdPos1=0;

KpVel1=1000;
TiVel1=0;
TdVel1=0;

% 2 prismatic joint
KpPos2=50;
TiPos2=1/50;
TdPos2=0;

KpVel2=1000;
TiVel2=0;
TdVel2=0;

% 3 rotational joint
KpPos3=50;
TiPos3=1/50;
TdPos3=0;

KpVel3=1000;
TiVel3=0;
TdVel3=0;

tt_end=8.8;
%% Simscape simulation without control 

% sim("Project_NoControl.slx")
h16=figure(16); % plot the torques of each joint
set(h16,'name','Motor torques comparison')
subplot(3,1,1);  % torque motor 1
    plot(tt,tor(1,:),'g','LineWidth',2);
    hold on
    plot(out.tor1_sim,'LineWidth',1.1)
    title('Motor 1'); 
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best')  
    xlabel('Tempo [s]')
    ylabel('Coppia [N*m]')
subplot(3,1,2);  % torque motor 2
    plot(tt,tor(2,:),'g','LineWidth',2); 
    hold on
    plot(out.tor2_sim,'LineWidth',1.1)
    title('Motor 2'); 
    xlabel('Tempo [s]')
    ylabel('Forza [N]')
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 
subplot(3,1,3);  % torque motor 3
    plot(tt,tor(3,:),'g','LineWidth',2); 
    hold on
    plot(out.tor3_sim,'LineWidth',1.1)
    title('Motor 3');
    xlabel('Tempo [s]')
    ylabel('Coppia [N*m]')
    grid on;
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 

h18=figure(18); % plot the gripper position
set(h18,'name','Gripper positions comparison')
subplot(3,1,1);  % x
    plot(tt,x,'g','LineWidth',2);
    hold on
    plot(out.x_sim1,'LineWidth',1.1)
     
    xlabel('Tempo [s]')
    ylabel('X [rad]')
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 
subplot(3,1,2);  % y
    plot(tt,y,'g','LineWidth',2); 
    hold on
    plot(out.y_sim1,'LineWidth',1.1)
    
    xlabel('Tempo [s]')
    ylabel('Y [m]')
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 
subplot(3,1,3);  % z
    plot(tt,z,'g','LineWidth',2); 
    hold on
    plot(out.z_sim1,'LineWidth',1.1)
     
    xlabel('Tempo [s]')
    ylabel('Z [rad]')
    grid on; 
    xlim([0 8.8])
    ylim([0.3 1.2])
legend('Theorical','SimScape','Location','best')

%% Simscape simulation with control 

% sim("Project.slx")
h17=figure(17); % plot the torques of each joint
set(h17,'name','Motor torques comparison')
subplot(3,1,1);  % torque motor 1
    plot(tt,tor(1,:),'g','LineWidth',1);
    hold on
    plot(out.tor1_csim,'LineWidth',1.1)
    title('Motor 1'); 
    xlabel('Tempo [s]')
    ylabel('Coppia [N*m]')
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 
subplot(3,1,2);  % torque motor 2
    plot(tt,tor(2,:),'g','LineWidth',1); 
    hold on
    plot(out.tor2_csim,'LineWidth',1.1)
    title('Motor 2'); 
    xlabel('Tempo [s]')
    ylabel('Forza [N]')
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 
subplot(3,1,3);  % torque motor 3
    plot(tt,tor(3,:),'g','LineWidth',1); 
    hold on
    plot(out.tor3_csim,'LineWidth',1.1)
    title('Motor 3');
    xlabel('Tempo [s]')
    ylabel('Coppia [N*m]')
    grid on; 
    xlim([0 8.8])
    ylim([6 10])
legend('Theorical','SimScape','Location','best') 

h18=figure(18); % plot the gripper position
set(h18,'name','Gripper positions comparison')
subplot(3,1,1);  % x
    plot(tt,x,'g','LineWidth',1);
    hold on
    plot(out.x_sim,'LineWidth',1.1)
     
    xlabel('Tempo [s]')
    ylabel('X [rad]')
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 
subplot(3,1,2);  % y
    plot(tt,y,'g','LineWidth',1); 
    hold on
    plot(out.y_sim,'LineWidth',1.1)
         
    xlabel('Tempo [s]')
    ylabel('Y [m]')
    grid on; 
    xlim([0 8.8])
    legend('Theorical','SimScape','Location','best') 
subplot(3,1,3);  % z
    plot(tt,z,'g','LineWidth',1); 
    hold on
    plot(out.z_sim,'LineWidth',1.1)
    xlabel('Tempo [s]')
    ylabel('Z [rad]')
    grid on; 
    xlim([0 8.8])
    ylim([0.3 1.2])
legend('Theorical','SimScape','Location','best')