function [ ang ] = angFromStdAng(robot,dh, isRight )
%Converts theta1:4 from DH to ang
% input is either a vector or a matrix. If it is a matrix then each row
% should be 4 theta values

if isRight
    index=8;    
else
    index=1;
end
if isvector(dh)
    N=1;
    nDof=numel(dh);
else
    nDof=size(dh,2);
    N=size(dh,1);
end
ang=dh;
if isvector(dh)    
    ang=sum(robot.B(index:index+nDof-1,:).*[dh(:),ones(nDof,1)],2);    
else
    for i=1:N
        ang(i,:)=sum(robot.B(index:index+nDof-1,:).*[dh(i,:)',ones(nDof,1)],2);
    end
end

end

