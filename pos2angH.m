function [ang]=pos2angH(robot, Ahn)
% finds head angles given the location of the head in neck coordinates
%   Detailed explanation goes here

Ahn=Ahn./norm(Ahn);
ang=zeros(2,1);

ang(1)=atan2(-Ahn(1),Ahn(2))+pi/2;
ang(2)=atan2(-Ahn(3),sqrt(Ahn(1).^2+Ahn(2).^2))+pi/2;

end

