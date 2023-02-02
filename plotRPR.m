function [cfg1,cfg2,h1,h2,h3,h4,h5,h6]=plotRPR(Q,L,colore,fig,axs)

    figure(fig); % fig figure number
    q1 = Q(1); %R
    q2 = Q(2); %P
    q3 = Q(3); %R
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l5 = L(5);
    
    x1 = 0;
    y1 = 0;
    z1 = 0;
    
    x2 = 0;
    y2 = 0;
    z2 = l1;
    
    x3 = l2*cos(q1);
    y3 = l2*sin(q1);
    z3 = l1;
    
    x4 = l2*cos(q1)-l3*sin(q1);
    y4 = l2*sin(q1)+l3*cos(q1);
    z4 = l1;
    
    x5 = l2*cos(q1)-(l3+q2)*sin(q1);
    y5 = l2*sin(q1)+(l3+q2)*cos(q1);
    z5 = l1;
    
    x6= l2*cos(q1)-(l3+q2+l5*cos(q3))*sin(q1);
    y6= l2*sin(q1)+(l3+q2+l5*cos(q3))*cos(q1);
    z6= l1+l5*sin(q3);

    hold on
    cfg1=plot3([x1 x2 x3 x4 x5 x6],[y1 y2 y3 y4 y5 y6],[z1 z2 z3 z4 z5 z6],'LineWidth',2,'color',colore); %manipulator
    cfg2=plot3([x2 x4 x5],[y2 y4 y5],[z2 z4 z5],'o','color',colore); %joint position
    grid on
    axis equal
    
    % draw reference systems
    h1=triad('Parent',axs,'Scale',0.05,'LineWidth',0.5,'Tag','','Matrix',makehgtform('translate',[0,0,0]));
    h2=triad('Parent',axs,'Scale',0.05,'LineWidth',0.5,'Tag','','Matrix',makehgtform('zrotate',q1,'translate',[0,0,l1]));
    h3=triad('Parent',axs,'Scale',0.05,'LineWidth',0.5,'Tag','','Matrix',makehgtform('zrotate',q1,'translate',[l2,0,l1]));
    h4=triad('Parent',axs,'Scale',0.05,'LineWidth',0.5,'Tag','','Matrix',makehgtform('zrotate',q1,'translate',[l2,l3,l1]));
    h5=triad('Parent',axs,'Scale',0.05,'LineWidth',0.5,'Tag','','Matrix',makehgtform('zrotate',q1,'translate',[l2,l3+q2,l1]));
    h6=triad('Parent',h5,'Scale',0.05,'LineWidth',0.5,'Tag','','Matrix',makehgtform('zrotate',0,'xrotate',q3,'translate',[0,l5,0]));

end