function [J] = inertia1(jx2,jy2,jz2,m2,l2,g2,jx3,jy3,jz3,m3,l3,g3)
Ixx2=(-jx2+jy2+jz2)/2;
Iyy2=(jx2-jy2+jz2)/2;
Izz2=(jx2+jy2-jz2)/2;

Ixx3=(-jx3+jy3+jz3)/2;
Iyy3=(jx3-jy3+jz3)/2;
Izz3=(jx3+jy3-jz3)/2;

Jb2=[Ixx2, 0, 0, 0;
   0, Iyy2, 0, 0;
   0, 0, Izz2, 0;
   0, 0, 0, m2];

Jb3=[Ixx3, 0, 0, 0;
   0, Iyy3, 0, 0;
   0, 0, Izz3, 0;
   0, 0, 0, m3];

M3=[1, 0, 0, 0;
   0, 1, 0, -(l3-g3);
   0, 0, 1, 0;
   0, 0, 0, 1];
J3=M3*Jb3*M3';

M2=[1, 0, 0, -(l2-g2);
   0, 1, 0, -l3;
   0, 0, 1, 0;
   0, 0, 0, 1];
J2=M2*Jb2*M2';

J=J2+J3;
end