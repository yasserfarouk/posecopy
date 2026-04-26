function [ Net, Nht ] = retarget(robot, Ast, Aet,Aht,isRight )
%function [ Net, Nht ] = retarget(robot, Ast, Aet,Aht,isRight )
%Converts positions relative to any skeleton to NAO coordinates
%   Detailed explanation goes here
Aes=Aet-Ast;
Ahs=Aht-Ast;
Ahe=Ahs-Aes;
Nes=robot.UpperArmLength*(Aes./norm(Aes));
Nhe=(robot.LowerArmLength)*(Ahe./norm(Ahe));
Nhs=Nes+Nhe;
if isRight   
    Nst=[0;robot.ShoulderOffsetYR;robot.ShoulderOffsetZR];    
else    
    Nst=[0;robot.ShoulderOffsetYL;robot.ShoulderOffsetZL];        
end
Net=Nst+Nes;
Nht=Nst+Nhs;

end

