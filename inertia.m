function [J] = inertia(jx,jy,jz,m,l,g)
Ixx=(-jx+jy+jz)/2;
Iyy=(jx-jy+jz)/2;
Izz=(jx+jy-jz)/2;

Jb=[Ixx, 0, 0, 0;
   0, Iyy, 0, 0;
   0, 0, Izz, 0;
   0, 0, 0, m];
M=[1, 0, 0, 0;
   0, 1, 0, -(l-g);
   0, 0, 1, 0;
   0, 0, 0, 1];
J=M*Jb*M';

end