function [ ok ] = isInRange( robot,ang, isRight)
%ISINRANGE Summary of this function goes here
%   Detailed explanation goes here

if isRight
    index=8;    
else
    index=1;
end
epsilon=.25*pi/180;
nDof=size(ang,1);
ok= (ang>=robot.ranges(index:index+nDof-1,1)-epsilon) & ...
    (ang<=robot.ranges(index:index+nDof-1,2)+epsilon);
end

