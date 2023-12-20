function [ dh ] = angToStdAng(robot,ang, isRight )
%Converts theta1:4 from DH to ang
% input is either a vector or a matrix. If it is a matrix then each row
% should be 4 theta values

if isRight
    index=8;    
else
    index=1;
end
if isvector(ang)
    N=1;
    nDof=numel(ang);
else
    nDof=size(ang,2);
    N=size(ang,1);
end
dh=ang;
if isvector(ang)    
    dh=sum(robot.A(index:index+nDof-1,:).*[ang(:),ones(nDof,1)],2);    
else
    for i=1:N
        dh(i,:)=sum(robot.A(index:index+nDof-1,:).*[ang(i,:)',ones(nDof,1)],2);
    end
end

end

