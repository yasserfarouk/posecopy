function [ ang ] = orientHead(robot, R )
%ORIENTHEAD Summary of this function goes here
%   Detailed explanation goes here
ay=atan2(R(1,3),sqrt(R(1,1)*R(1,1)+R(1,2)*R(1,2)));
if abs(1-R(1,3))<0.01       %ay=pi/2
    % here we can only find az+ax but we have no x joint anyway
    ax=0;
    az=atan2(R(3,2),R(2,2));
elseif abs(1+R(1,3))<0.01   %ay=-pi/2
    % here we can only find az-ax but we have no x joint anyway
    ax=0;
    az=atan2(R(2,1),R(3,1));
else                        %ay~=+-pi/2
    % we need both rotation arund x and z but NAO cannot rotate around x so
    % we simply ignore the rotation around x
    %ax=atan2(-R(2,3),R(3,3));
    az=atan2(-R(1,2),R(1,1));
end

ang=[az;-ay];
end

