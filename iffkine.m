function [ Tht,Tet] = iffkine(robot,ang,isRight)
% function [ Aht,Aet] = ang2pos(robot,ang,isRight)
% applies forward kinematics
%
% robot     The nao object
% ang       N*4 angles in the order SP,SR,EY,ER
% isRight   if true then right hand values are returned
%
sang=robot.angToStdAng(ang,isRight);
theta=robot.sangToDHTheta(sang,isRight);
if isvector(theta)
    theta=theta(:)';
    sang=sang(:)';
end
if isRight
    Tht=robot.rightAll.fkine(theta(:,1:4));  
    if nargout>1       
        Tet=robot.rightUpper.fkine(sang(:,[1,2]));        
    end
else
    Tht=robot.leftAll.fkine(theta(:,1:4));  
    if nargout>1       
        Tet=robot.leftUpper.fkine(sang(:,[1,2]));        
    end
end