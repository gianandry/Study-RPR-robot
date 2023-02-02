function [L]=L_matrix(j)
    if j==1
        L=zeros([4,4]); L(1,2)=-1; L(2,1)=1;
    elseif j==2
        L=zeros([4,4]); L(2,4)=1; 
    elseif j==3
        L=zeros([4,4]); L(2,3)=-1; L(3,2)=1;
    end
end