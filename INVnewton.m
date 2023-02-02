function Qinv = INVnewton(S,Q0,L,tol,Nmax)

    S0=RPR_dir(Q0,L);
    iter=0;
    while norm(S-S0,2)>tol
        if iter>Nmax 
            break
        else
            J=RPR_jac(Q0,L);
            Q=Q0+J^(-1)*(S-S0);
            S0=RPR_dir(Q,L);
            iter=iter+1;
            Q0=Q;
        end
    end
    Qinv=Q0;
    
end