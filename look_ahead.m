function [t,v,a,vx,vy,ax,ay] = look_ahead(x,y,tini,vini,vfin,vmax,amax,approx,xc,yc,R,verso)
    % verso 1=antiorario, 2=orario
    t=zeros(1,approx); s=zeros(1,approx); v=zeros(1,approx); a=zeros(1,approx);
    t(1)=tini;
    for i=1:approx
        if i==1 
            v(i)=vini;
        else
            v_a=sqrt(v(i-1)^2 +2*amax*sqrt((x(i)-x(i-1))^2 +(y(i)-y(i-1))^2));
            v(i)=min(v_a,vmax);
        end
    
        if v(i)==vmax 
                a(i)=0;
            else
                a(i)=amax;
        end
    end

    for i=approx:-1:1
        if i==approx
            v(i)=min(v(approx),vfin);
            v_a=v(i);
        else
            v_a=sqrt(v(i+1)^2 +2*amax*sqrt((x(i)-x(i+1))^2 +(y(i)-y(i+1))^2));
            v(i)=min(v_a,v(i));
        end
    
        if v(i)<v_a
            continue
        end
    
        if v(i)==vmax 
                a(i)=0;
            else
                a(i)=-amax;
        end
    end
    
    for i=2:approx
        if v(i)==vmax
            t(i)=t(i-1)+sqrt((x(i)-x(i-1))^2 +(y(i)-y(i-1))^2)/v(i);
        else
            t(i)=t(i-1)+abs((v(i)-v(i-1))/a(i));
        end
        s(i)=s(i-1) +sqrt((x(i)-x(i-1))^2 +(y(i)-y(i-1))^2);
    end
    for i=1:approx
        if verso==1
            alfa(i)=atan2(y(i)-yc,x(i)-xc);
            vx(i)=v(i)*cos(pi/2+alfa(i));
            vy(i)=v(i)*sin(pi/2+alfa(i));
            at=a(i);
            an=v(i)^2/R;
            gamma=atan2(at,an);
            ax(i)=-sqrt(at^2+an^2)*cos(gamma-alfa(i));
            ay(i)=sqrt(at^2+an^2)*sin(gamma-alfa(i));
        elseif verso==2
            alfa(i)=atan2(y(i)-yc,x(i)-xc);
            vx(i)=v(i)*cos(pi/2-alfa(i));
            vy(i)=-v(i)*sin(pi/2-alfa(i));
            at=a(i);
            an=v(i)^2/R;
            gamma=atan2(at,an);
            ax(i)=sqrt(at^2+an^2)*sin(gamma+alfa(i)-pi/2);
            ay(i)=-sqrt(at^2+an^2)*cos(gamma+alfa(i)-pi/2);
        end
    end

end