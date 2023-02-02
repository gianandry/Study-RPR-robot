function J = RPR_jac(Q,L)

    q1 = Q(1); %R
    q2 = Q(2); %P
    q3 = Q(3); %R
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l5 = L(5);

    J=zeros(3,3);
    J(1,1) = -l2*sin(q1)-(l3+q2+l5*cos(q3))*cos(q1);   
    J(1,2) = -sin(q1);
    J(1,3) = l5*sin(q1)*sin(q3);
    J(2,1) = l2*cos(q1)-(l3+q2+l5*cos(q3))*sin(q1);
    J(2,2) = cos(q1);
    J(2,3) = -l5*cos(q1)*sin(q3);
    J(3,1) = 0;
    J(3,2) = 0;
    J(3,3) = l5*cos(q3);

end

