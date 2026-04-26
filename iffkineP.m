function [ Aht,Aet] = iffkineP(robot,ang,isRight)
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
%notice that for the upper robot model \theta^U=ang and this is why the
%upper body model is always using ang rather than theta
if isRight
    Tht=robot.rightAll.fkine(theta(:,[1,2,3,4]));    
    Aht=squeeze(Tht(1:3,4));
    if nargout>1
        Tet=robot.rightUpper.fkine(sang(:,[1,2]));    
        Aet=squeeze(Tet(1:3,4));
    end
else
    Tht=robot.leftAll.fkine(theta(:,[1,2,3,4]));    
    Aht=squeeze(Tht(1:3,4));
    if nargout>1
        Tet=robot.leftUpper.fkine(sang(:,[1,2]));    
        Aet=squeeze(Tet(1:3,4));
    end
end