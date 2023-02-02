function q = RPR_inv(s,L)

    x = s(1); %R
    y = s(2); %P
    z = s(3); %R
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l4 = L(4);
    l5 = L(5);

    q(3)=asin((z-l1)/l5);
    q(2)=(x^2 +y^2-l2^2)^0.5-l3-l5*cos(q(3));
    q(1)=atan2(y,x)-atan2(l5*cos(q(3))+q(2)+l3,l2);

end

