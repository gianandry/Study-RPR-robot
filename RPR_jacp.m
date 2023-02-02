function Jp = RPR_jacp(Q,Qp,L)
    
    q1 = Q(1); %R
    q2 = Q(2); %P
    q3 = Q(3); %R
    qp1= Qp(1);
    qp2= Qp(2);
    qp3= Qp(3);
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l5 = L(5);

    Jp=zeros(3,3);
    Jp(1,1) = -l2*cos(q1)*qp1+sin(q1)*qp1*(l3+q2+l5*cos(q3))-cos(q1)*(qp2-l5*qp3*sin(q3));  
    Jp(1,2) = -cos(q1)*qp1;
    Jp(1,3) = l5*qp1*cos(q1)*sin(q3)+l5*sin(q1)*qp3*cos(q3);
    Jp(2,1) = -l2*sin(q1)*qp1-cos(q1)*qp1*(l3+q2+l5*cos(q3))-sin(q1)*(qp2-l5*qp3*sin(q3)); 
    Jp(2,2) = -sin(q1)*qp1; 
    Jp(2,3) = l5*sin(q3)*qp1*sin(q1)-l5*cos(q3)*qp3*cos(q1);
    Jp(3,1) = 0;
    Jp(3,2) = 0;
    Jp(3,3) = -l5*sin(q3)*qp3;

end

