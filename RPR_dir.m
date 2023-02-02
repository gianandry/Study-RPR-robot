function s=RPR_dir(Q,L)
    
    q1 = Q(1); %R
    q2 = Q(2); %P
    q3 = Q(3); %R
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l5 = L(5);
    
    s=zeros(3,1);
    s(1)=l2*cos(q1)-(l3+q2+l5*cos(q3))*sin(q1); % x
    s(2)=l2*sin(q1)+(l3+q2+l5*cos(q3))*cos(q1); % y
    s(3)=l1+l5*sin(q3); % z
    
end