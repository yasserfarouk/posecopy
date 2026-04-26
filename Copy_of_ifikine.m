function [ ang,singularityType,error] = ifikine(robot,Aet,Aht,isRight,ignoreHandTransformation)
% function [ ang,singularityType,theta ] = pos2ang(robot,Aet,Aht,isRight,exactElbowYaw)
% Cacluates joint angles (Shoulder Pitch and Roll and Elbow Roll and Yaw)
% given positions of elbow and extremety in shoulder coordinates. Aet and
% Aht MUST be vectors not matrices
%   Aes             Position of Elbow in torso coordinates
%   Aht             Position of Hand in torso coordinates
%   isRight         pasShoulderWristd as 1 if this is the right part of the robot
%   exactElbowYaw   if true then a numerical step is applied at the end to
%           get exact value for elbowYaw otherwise an approximate value is
%           returned (default is false)
% Output:
% ======
% ang are the four angles of the robot arm in this order:
%  'ShoulderPitch'; 'ShoulderRoll'; 'ElbowYaw';'ElbowRoll'
%
% singularityType   0 No singularity
%                   1 theta(2)=pi/2   for left arm
%                   1 theta(2)=-pi/2  for right arm
%                   2 thetaL(2)=0
%                   4 lower arm extended
%                   8 unreachable because it requires a longer lower arm.
%                   We just reach in the same direction
%                   combinations mean combinations
%  Ajk  for translation of joint j in frame k
%  Dij  distance between joints i,j
%  Rij  rotation of the coordinate frame in i in the frame j
%  T    transformation (4*4) of the coordinate frame in i in the frame j
if ~exist('ignoreHandTransformation','var')
    ignoreHandTransformation=false;
end
singularityType=0;
%theta=zeros(4,2);
thetaU=zeros(2,1);
thetaL=zeros(2,2);
Aet=[Aet(:);1];
Aht=[Aht(:);1];
smallLength=robot.smallLength;
if isRight
    Aes=robot.Rts*Aet;
else
    Aes=robot.Lts*Aet;
end

if abs(Aes(1))<smallLength && abs(Aes(2))<smallLength
    % theta(2) is (+-)pi/2. In this case we cannot find theta (1)
    % from upperarm alone but it affects the hand position

    % notice that in this case we may have multiple possible
    % combinations of theta(1),thetaL(1),thetaL(2) to get the same elbow
    % and hand positions!!!

    % we arbitrary set theta(1)=0 in this cases
    singularityType=1;        
    thetaU(2)=asin(Aes(3)/robot.UpperArmLength);        
    thetaU(1)=0.0;      
    %this may be corrected later using hand coordinates
    % but only if we have a relative oreintation of the hand not only
    % position
    % when we have this singularity the axis of theta(1) and theta(3) is the
    % same so once we find theta(3) later in this function we are actually
    % finding theta(1)+theta(3). If we found that theta(3) is near its limits,
    % we can simply modify theta(1) and theta(3) as we please keeping their
    % sum.
else
    % theta(2) is not (+-)pi/2. In this case we can easily find
    % theta (1) then theta(2)
    thetaU(1)=atan2(Aes(2),Aes(1));        
    thetaU(2)=atan2(Aes(3),mysqrt(Aes(2).^2+Aes(1).^2));
end

% now we find the orientation of the elbow in the torso coordinates
% we calculate in the upper arm robot
if isRight
    Leu_t=squeeze(robot.rightUpper.fkine(thetaU([1,2])));
else
    Leu_t=squeeze(robot.leftUpper.fkine(thetaU([1,2])));
end

%
L=robot.LowerArmLength;
if ignoreHandTransformation
    Lz=0;
    Ly=0;
else
    Lz=robot.HandOffsetZ;    
    Ly=robot.HandOffsetY;
end

% make a transformation from torso coordinate to elbow coordinate in
% the full robot by taking the transformation from torso to elbow in
% upper arm (LUte=Hinv(LUet))
Lt_eu=humanoid.Hinv(Leu_t);    
Ahe=Lt_eu*Aht;    
% notice that the forward kinematics of the lower arm assumes that the
% base is in the same transformation as the end effector of the upper
% arm.
x=Ahe(1); 
if L<abs(x)
    % here means that the hand location cannot be reached at
    % all because it needs a longer lower-arm
    % we just move in its direction
    Ahe=L.*(Ahe./norm(Ahe));            
    singularityType=8;    
    x=Ahe(1); 
end
y=Ahe(2); z=Ahe(3);
% a small error in position data may cause lArmR to be >1 which
% leads to a complex thata(4). We avoid that for small values
nSolutions=1;
thetaL=zeros(2,nSolutions);
if (L-abs(x))<1e-5
    % this means that the arm is extended and so thetaL(1) has no 
    % effect on the position of the hand (or the elbow of course)             
    singularityType=singularityType+4;     
    if x>0
        % this means that the angle is extended forward
        c2=0; s2=-1;        
        if (abs(Lz)<smallLength && abs(Ly)<smallLength)
            thetaL(1)=0;
        else
            c1=(z*Lz+y*Ly)/(Ly^2+Lz^2);
            if(abs(Ly)>abs(Lz))                        
                s1=(z-Lz*c1)/Ly;                
            else                
                s1=(Ly*c1-y)/Lz;
            end
            thetaL(1)=atan2(s1,c1);
        end
    else
        c2=0; s2=1;        
        if (abs(Lz)<smallLength && abs(Ly)<smallLength)
            thetaL(1)=0;
        else
            c1=(z*Lz-y*Ly)/(Ly^2+Lz^2);
            if(abs(Ly)>abs(Lz))                
                s1=(Lz*c1-z)/Ly;
            else                        
                s1=-(y+Ly*c1)/Lz;
            end
            thetaL(1)=atan2(s1,c1);
        end                
    end
    thetaL(2)=atan2(s2,c2);        
%     s41=(L+sqrt(L^2+(Ly^2+1)*(L^2-Ly^2)))/(L^2-Ly^2);
%     s42=(L-sqrt(L^2+(Ly^2+1)*(L^2-Ly^2)))/(L^2-Ly^2);
%     theta(4)=findCorrectS4(s41,s42,L,Ly,Lz,x,y,z);        
%     theta(3)=0.0;
else            
    % the arm is not extended
    nSolutions=2;    
    %notice that theta(4) has two solutions and we will try both
    if abs(Ly)<smallLength
        c2(1)=mysqrt(y^2+z^2-Lz^2)/L;
        c2(2)=-c2(1);
        s2(1)=-x/L;        
        s2(2)=s2(1);        
    else
        gamma=y^2+z^2-Ly^2-Lz^2;
        tmpc=(mysqrt(gamma*(L^2)+(x^2+gamma)*(Ly^2)))/(L^2+Ly^2);
        c2(1)=(-Ly*x)/(L^2+Ly^2)+tmpc;
        c2(2)=c2(1)-2*tmpc;
        s2(1)=-(x+Ly*c2(1))/L;        
        s2(2)=-(x+Ly*c2(2))/L;               
    end
    for sol=1:2     
%         if isRight
%             thetaL(2,sol)=atan2(s2(3-sol),c2(3-sol));
%         else
%             thetaL(2,sol)=atan2(s2(sol),c2(sol));
%         end
        thetaL(2,sol)=atan2(s2(sol),c2(sol));
        b=Lz; 
        a=L*c2(sol)-Ly*s2(sol);
        s1=(a*z-b*y)/(a^2+b^2);
        if(abs(a)>abs(b))        
            c1=(y+b*s1)/a;
        else        
            c1=(z-a*s1)/b;
        end        
        thetaL(1,sol)=atan2(s1,c1);        
    end    
end
% we have now two solutions for thetaL and one for thetaU
if nSolutions>1    
    theta1=zeros(4,nSolutions);
    ang1=zeros(4,nSolutions);
    ok=zeros(nSolutions,1);
    for sol=1:nSolutions
        theta1(:,sol)=humanoid.thetaFromThetaUL(thetaU,thetaL(:,sol),isRight);        
        ang1(:,sol)=robot.angFromDHTheta(adjustAngle(theta1(:,sol)),isRight);
        tmp=robot.isInRange(ang1(:,sol),isRight);
        ok(sol)=tmp(4);
    end
    testForward=false;
    if ok(1) && ~ ok(2)
        ang=ang1(:,1);        
    elseif ~ok(1) && ok(2)
        ang=ang1(:,2);        
    else
        testForward=true;
    end

    if nargout>2 || testForward    
        if isRight
            Tht=robot.rightAll.fkine(theta1(:,1));    
            Aht1=squeeze(Tht(1:3,4));        
            Tht=robot.rightAll.fkine(theta1(:,2));    
            Aht2=squeeze(Tht(1:3,4));        
        else
            Tht=robot.leftAll.fkine(theta1(:,1));    
            Aht1=squeeze(Tht(1:3,4));    
            Tht=robot.leftAll.fkine(theta1(:,2));    
            Aht2=squeeze(Tht(1:3,4));    
        end
        err1=norm(Aht1-Aht(1:3));
        err2=norm(Aht2-Aht(1:3));        
        if testForward
            if err1<err2
                ang=ang1(:,1);
            else
                ang=ang1(:,2);
            end
        else
            if ok(1)
                err2=err1*100;
            else
                err1=err2*100;
            end
        end
        if nargout>2
            error=min(err1,err2); 
        end
    end
else
    theta=humanoid.thetaFromThetaUL(thetaU,thetaL(:,1),isRight);
    ang=robot.angFromDHTheta(theta,isRight);
    if nargout>2
        if isRight
            Tht=robot.rightAll.fkine(theta);    
            Aht1=squeeze(Tht(1:3,4));                    
        else
            Tht=robot.leftAll.fkine(theta);    
            Aht1=squeeze(Tht(1:3,4));                
        end
        error=norm(Aht1-Aht(1:3));
    end
end
ang=adjustAngle(ang);
%ang=putInRange(robot,ang,isRight);
end
function theta=adjustAngle(theta)
    theta=theta+pi;
    theta=theta-(2*pi).*floor(theta./(2*pi));    
    theta=theta-pi;
    theta(isnan(theta))=-inf;    
% end
end
function ang=adjustAngle2(ang)
    ang(ang<-pi)=ang(ang<-pi)+2*pi;
    ang(ang>pi)=ang(ang>pi)-2*pi;
end
function ang=putInRange(robot,ang,isRight)
    if isRight
        index=8;    
    else
        index=1;
    end
    nDof=size(ang,1);
    for i=1:nDof
%         while ang(i)<robot.ranges(i-1+index,1)
%             ang(i)=ang(i)+2*pi;
%         end
%         while ang(i)>robot.ranges(i-1+index,2)
%             ang(i)=ang(i)-2*pi;
%         end
        if ang(i)<robot.ranges(i-1+index,1)
            ang(i)=robot.ranges(i-1+index,1);
        elseif ang(i)>robot.ranges(i-1+index,2)
            ang(i)=robot.ranges(i-1+index,2);
        end
    end    
end
function b=mysqrt(a)
b=sqrt(max(a,0));
end