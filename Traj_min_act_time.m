function [tt,q1,q2,q3,qp1,qp2,qp3,qpp1,qpp2,qpp3] = Traj_min_act_time(qini,qfin,qpmax,qppmax,approx)
    delta_q=qfin-qini;

    for i=1:3
        delta_q_lim=(qpmax(i)^2)/qppmax(i);
        if delta_q_lim<abs(delta_q(i)) % arriva a qpmax
            t_att(i)=abs(delta_q(i))/qpmax(i)+qpmax(i)/qppmax(i);
            t1(i)=qpmax(i)/qppmax(i);
        else
            t_att(i)=sqrt(4*abs(delta_q(i))/qppmax(i));
            t1(i)=t_att(i)/2;
        end
    end
 
    T=max(t_att);
    for i=1:3
        lambda(i)=t1(i)/T;
    end
     n=find(t_att==T);

    for i=1:approx
        t=(i-1)*T/(approx-1); % time from 0 to T with step dT
        tt(i)=t;
        
%         [q1(i),qp1(i),qpp1(i)]=tretratti(t,T,qini(1),delta_q(1),lambda(1),lambda(1));
%         [q2(i),qp2(i),qpp2(i)]=tretratti(t,T,qini(2),delta_q(2),lambda(2),lambda(2));
%         [q3(i),qp3(i),qpp3(i)]=tretratti(t,T,qini(3),delta_q(3),lambda(3),lambda(3));
        [q1(i),qp1(i),qpp1(i)]=tretratti(t,T,qini(1),delta_q(1),lambda(n),lambda(n));
        [q2(i),qp2(i),qpp2(i)]=tretratti(t,T,qini(2),delta_q(2),lambda(n),lambda(n));
        [q3(i),qp3(i),qpp3(i)]=tretratti(t,T,qini(3),delta_q(3),lambda(n),lambda(n));
    end
end
