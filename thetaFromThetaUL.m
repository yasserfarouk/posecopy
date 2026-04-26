function [ theta ] = thetaFromThetaUL(thetaU,thetaL, isRight )
%THETAFROMTHETAUL Summary of this function goes here
%   Detailed explanation goes here
theta=[thetaU;thetaL];
theta(2,:)=thetaU(2,:)+pi/2;
end

