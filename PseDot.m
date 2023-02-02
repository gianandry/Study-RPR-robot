function t = PseDot(A,B)
%PseDot Pseudo Dot Product
t=A(1,4)*B(1,4)+A(2,4)*B(2,4)+A(3,4)*B(3,4)+A(1,2)*B(1,2)+A(1,3)*B(1,3)+A(2,3)*B(2,3);
end
