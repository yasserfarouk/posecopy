function [ ang,singularityType,err] = ifikine(robot,Aet,Aht,isRight,ignoreHandTransformation)
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

theta=zeros(4,2);       %we can have a maximum of 2 solutions
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
    
    % we know that thetaU(2)=pi/2 or pi-pi/2 which is the same
    % we arbitrary set theta(1)=0 in this cases
    singularityType=1;        
    theta(2,1)=pi/2;%asin(Aes(3)/robot.UpperArmLength);        
    theta(1,1)=0.0;      
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
    theta(1,1)=atan2(Aes(2),Aes(1));        
    theta(2,1)=atan2(Aes(3),mysqrt(Aes(2).^2+Aes(1).^2));
end
theta(:,2)=theta(:,1);
% now we find the orientation of the elbow in the torso coordinates
% we calculate in the upper arm robot
% we know that thataU(1,2)=theta(1,2)
if isRight
    Leu_t=squeeze(robot.rightUpper.fkine(theta([1,2],1)));
else
    Leu_t=squeeze(robot.leftUpper.fkine(theta([1,2],1)));
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
    singularityType=singularityType+8;
    x=Ahe(1); 
end
y=Ahe(2); z=Ahe(3);
xhat=x/L;
% a small error in position data may cause lArmR to be >1 which
% leads to a complex thata(4). We avoid that for small values
nSolutions=1;
if (abs(Ly)<1e-5 && abs(Lz)<1e-5)
    if abs(xhat)<smallLength*1e-3
        % extended arm
        nSolutions=1;
        singularityType=singularityType+4;        
        theta(4,1)=atan2(0,sign(xhat));
        theta(3,1)=0.0;
    else
        nSolutions=2;
        p=mysqrt(y^2+z^2);
        theta(4,1)=atan2(p,x);
        theta(4,2)=atan2(-p,x);
        theta(3,:)=atan2(y*(sin(theta(4,:)).^-1),z*(sin(theta(4,:)).^-1));
    end
else
    c4=zeros(2,1);
    s4=zeros(2,1);
    c3=zeros(2,1);
    s3=zeros(2,1);
    if (abs(Ly)>1e-5 && abs(Lz)>1e-5)
        nSolutions=2;
        gama=y^2+z^2-L^2-Lz^2;
        p1=(x*L)/(L^2+Ly^2);
        sqrtPart=mysqrt(x^2*L^2+gama*(L^2+Ly^2))/(L^2+Ly^2);
        c4(1)=p1+sqrtPart;
        c4(2)=p1-sqrtPart;
        s4=(L.*c4-x)./Ly;
        for i=1:2
            a=Ly*c4(i)+L*s4(i);
            b=Lz;
            c3(i)=(a*y+b*z)/(a^2+b^2);
            if abs(a)>abs(b)
                s3(i)=(z-b*c3(i))/a;
            else
                s3(i)=(a*c3(i)-y)/b;
            end
        end
    elseif (abs(Lz)>1e-5)
        nSolutions=2;
        c4(1)=xhat;
        c4(2)=c4(1);
        s4(1)=mysqrt(1-c4(1)^2);
        s4(2)=-s4(1);
        a=L.*s4;
        b=Lz;
        for i=1:2
            c3(i)=(a(i)*y+b*z)/(a(i)^2+b^2);
            if abs(a(i))>abs(b)
                s3(i)=(z-b*c3(i))/a(i);
            else
                s3(i)=(a(i)*c3(i)-y)/b;
            end
        end  
    end
    for i=1:nSolutions
        theta(3,i)=atan2(s3(i),c3(i));
        theta(4,i)=atan2(s4(i),c4(i));
    end
end

% we have now two solutions for thetaL and one for thetaU
%theta=adjustthetale(theta);
testForward=false;
ang=robot.angFromStdAng(theta,isRight);
if nSolutions>1            
    ok=zeros(nSolutions,1);
    for sol=1:nSolutions                
        tmp=robot.isInRange(ang(:,sol),isRight);
        ok(sol)=tmp(4);
    end
    testForward=false;
    if ok(1) && ~ ok(2)
        ang=ang(:,1);        
        nSolutions=1;
    elseif ~ok(1) && ok(2)
        ang=ang(:,2);        
        nSolutions=1;
    else
        %prefer solutions that are achievable by human elbow
       for sol=1:nSolutions
            ok(sol)=(isRight && ang(4,sol)>0 )||((~isRight) && ang(4,sol)<0);
       end 
       if ok(1) && ~ ok(2)
            ang=ang(:,1);        
            nSolutions=1;
        elseif ~ok(1) && ok(2)
            ang=ang(:,2);        
            nSolutions=1;
       else
           testForward=true;
       end   
    end
end

ang=putInRange(robot,ang,isRight);
if nargout>2 || testForward    
    err=zeros(nSolutions,1);
    for i=1:nSolutions
        Aht1=robot.iffkineP(ang(:,i)',isRight);
        err(i)=norm(Aht1-Aht(1:3));
    end  
    if nSolutions>1        
        [err,errI]=min(err);
        ang=ang(:,errI);                    
    end
end
%ang=adjustAngle(ang);
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