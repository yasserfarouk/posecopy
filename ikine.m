function [ ang,singularityType,err] = ikine(robot,Tht,isRight,angHint)
    singularityType=-1;
    angHint=robot.angToStdAng(angHint,isRight);
    angHint=robot.sangToDHTheta(angHint,isRight);
    if isRight
        theta=robot.rightAll.ikine(Tht,angHint,[1,1,1,1,0,0],'verbose',0,'alpha',0.25,'pinv');        
    else
        theta=robot.leftAll.ikine(Tht,angHint,[1,1,1,1,0,0],'verbose',0,'alpha',0.25,'pinv');
    end
    if sum(isnan(theta))
        warning('nanTheta',sprintf('Warning: some angles are nans %f',theta));
        singularityType=-2;
    end
    theta(isnan(theta))=0;
    ang=robot.sangFromDHTheta(theta,isRight);
    ang=robot.angFromStdAng(ang,isRight);
    ang=putInRange(robot,ang,isRight);
    if nargout>2         
        Aht1=robot.iffkineP(theta,isRight);
        err=norm(Aht1-Tht(1:3,4)); 
    end
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

