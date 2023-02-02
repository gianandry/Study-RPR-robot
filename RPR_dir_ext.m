function Se = RPR_dir_ext(Q,L,G)
    
    q1 = Q(1); %R
    q2 = Q(2); %P
    q3 = Q(3); %R
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l4 = L(4);
    l5 = L(5);
    g1 = G(1);
    g2 = G(2);
    g3 = G(3);
    g4 = G(4);
    g5 = G(5);

    Se=zeros(18,1);
    Se(1) = l2*cos(q1)-(l3+q2+l5*cos(q3))*sin(q1);   
    Se(2) = l2*sin(q1)+(l3+q2+l5*cos(q3))*cos(q1);
    Se(3) = l1+l5*sin(q3);
    Se(4) = l2*cos(q1)-(l3+q2+g5*cos(q3))*sin(q1);   
    Se(5) = l2*sin(q1)+(l3+q2+g5*cos(q3))*cos(q1);
    Se(6) = l1+g5*sin(q3);
    Se(7) = l2*cos(q1)-(l3+q2-(l4-g4))*sin(q1);
    Se(8) = l2*sin(q1)+(l3+q2-(l4-g4))*cos(q1);
    Se(9) = l1;
    Se(10) = l2*cos(q1)-g3*sin(q1);
    Se(11) = l2*sin(q1)+g3*cos(q1);
    Se(12) = l1;
    Se(13) = g2*cos(q1);  
    Se(14) = g2*sin(q1);
    Se(15) = l1;
    Se(16) = q1;   
    Se(17) = q1+pi/2;
    Se(18) = q3;
    
end