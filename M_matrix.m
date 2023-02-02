function [M] = M_matrix(Q,L,j)

    q1 = Q(1); %R
    q2 = Q(2); %P
    q3 = Q(3); %R
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l5 = L(5);
   
    if j==1
        M=[cos(q1), -sin(q1), 0, l2*cos(q1)-l3*sin(q1);
           sin(q1), cos(q1), 0, l2*sin(q1)+l3*cos(q1);
           0, 0, 1, l1;
           0, 0, 0, 1];
    
    elseif j==2
        M=[1, 0, 0, 0;
           0, 1, 0, q2;
           0, 0, 1, 0;
           0, 0, 0, 1];
    
    elseif j==3
        M=[1, 0, 0, 0;
           0, cos(q3), -sin(q3), l5*cos(q3);
           0,  sin(q3), cos(q3), l5*sin(q3);
           0, 0, 0, 1];
    end

end