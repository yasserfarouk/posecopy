function [ thetaU,thetaL ] = thetaToThetaUL (theta, isRight )
%THETATOTHETAUL Summary of this function goes here
%   Detailed explanation goes here
thetaU=theta(1:2,:);
thetaL=theta(3:4,:);
thetaU(2,:)=theta(2,:)-pi/2;

end

