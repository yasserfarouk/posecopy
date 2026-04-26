classdef humanoid    
    properties(SetAccess=public)        
        model               = 'nao3.2';
        name                = 'nao3.2';        
        ranges=(pi/180).*[     -119.5,119.5;
                               .5,94.5;
                               -119.5,119.5;
                               -89.5,-.5;
                               -104.5,104.5;
                               0,0;
                               0,0;
                               -119.5,119.5;
                               -94,-.5;
                               -119.5,119.5;
                               .5,89.5;
                               -104.5,104.5;
                               0,0;
                               0,0];        
        smallLength=5e-3;
        zerosAng            = (pi/180).*[0,.5,0,-.5,0,0,0,0,-.5,0,.5,0,0,0];
    end
%     properties(Constant)
%         handJointNames={'LShoulderPitch'; 'LShoulderRoll'; 'LElbowYaw';'LElbowRoll';'LWristYaw';...
%                         'RShoulderPitch'; 'RShoulderRoll'; 'RElbowYaw';'RElbowRoll';'RWristYaw'};
%     
%         torsoIndex          =1;
%         spineIndex          =2;
%         shoulderCenterIndex =3;
%         headIndex           =4;
%         lShoulderIndex      =5;
%         lElbowIndex         =6;
%         lWristIndex         =7;
%         lHandIndex          =8;
%         rShoulderIndex      =9;
%         rElbowIndex         =10;
%         rWristIndex         =11;
%         rHandIndex          =12;        
%     end
    properties(Access=public)        
        ShoulderOffsetXL 	= 0.0;
        ShoulderOffsetYL 	= 98.00/1000;
        ShoulderOffsetZL	= 100.0/1000;
        ShoulderOffsetXR 	= 0.0;
        ShoulderOffsetYR 	= -98.00/1000;
        ShoulderOffsetZR	= 100.0/1000;
        ElbowOffsetY		= 0.0;
        UpperArmLength		= 90.0/1000;     %remember to correct arm length        
        LowerArmLength		= 108.55/1000;%remember to correct arm length        
        HandOffsetY			= 0.0/1000;%remember to correct arm length
        HandOffsetZ			= -15.9/1000;
        ArmLength           = (90.0+108.55)/1000;                
        A                   = [ones(14,1),zeros(14,1)];
        B                   = [ones(14,1),zeros(14,1)];
    end    
    properties(Access=protected)
        isPanTiltShoulder   = true;
        ThetaU2Theta        = [1,0;1,pi/2];
        ThetaL2Theta        = [1,0;1,0];
        leftUpper
        rightUpper
        leftAll
        rightAll
        Lst             %initial transformation matrix from torso to left shoulder
        Lts             % inverse of Lst
        Rst             %initial transformation matrix from torso to left shoulder
        Rts             % inverse of Lst
        Loh             %final transformation matrix to correct for 15.9 error in 3.2 robots
        Roh             %final rotation of the hand
        Lle_ue
        Lue_le
        Lue_e        
   end
   methods
       function robot=humanoid(isPanTiltShoulder,S,U,L,H,R,A,model,name)
           % isPanTiltShoulder      if true a PanTiltShoulder robot
           %                        otherwise a Spherical Shoulder one
           % S                      if a vector then it is the (xyz)
           %                        distance from torso center to left and
           %                        right shoulder. It is assumed that the
           %                        right shoulder is x,-y,z.
           %                        if 3*2 then it is the independent
           %                        translations from the torso to left and
           %                        right shoulders in order           
           % U                      Upper arm length
           % L                      lower arm length
           % H                      translation from the end of the wrist
           %                        to the arm's end effector
           % R                      14*2 matrix giving the ranges of all
           %                        left then right arm DoFs. If a DoF does
           %                        not exist pass its minimum=max=0
           % A                      14*2 matrix giving values ai,bi where
           %                        angleAsInRoboti=ai*Theta+bi
           % model                  string giving the robot's model name
           % name                   string giving the robot's name
           
           if exist('isPanTiltShoulder','var')
                robot.isPanTiltShoulder=isPanTiltShoulder;           
           end 
           if exist('S','var')
               if isvector(S)
                    robot.ShoulderOffsetXL=S(1);
                    robot.ShoulderOffsetYL=S(2);
                    robot.ShoulderOffsetZL=S(3);
                    
                    robot.ShoulderOffsetXR=S(1);
                    robot.ShoulderOffsetYR=-S(2);
                    robot.ShoulderOffsetZR=S(3);
               else
                    robot.ShoulderOffsetXL=S(1,1);
                    robot.ShoulderOffsetYL=S(2,1);
                    robot.ShoulderOffsetZL=S(3,1);

                    robot.ShoulderOffsetXR=S(1,2);
                    robot.ShoulderOffsetYR=S(2,2);
                    robot.ShoulderOffsetZR=S(3,2);
               end
           end           
           if exist('U','var')
                robot.UpperArmLength		= U;%remember to correct arm length        
           end
           if exist('L','var')
                robot.LowerArmLength		= L;%remember to correct arm length        
           end
           if exist('H','var')
               if isvector(H)
                    if abs(H(1))>0.00001
                        robot.LowerArmLength=robot.LowerArmLength+H(1);
                        H(1)=0;
                    end
                    robot.HandOffsetY		= H(2);%remember to correct arm length
                    robot.HandOffsetZ		= H(3);
               else
                   if abs(H(1))>0.00001
                        robot.LowerArmLength=robot.LowerArmLength+H(1,4);
                        H(1,4)=0;
                    end
                    robot.HandOffsetY		= H(2,4);%remember to correct arm length
                    robot.HandOffsetZ		= H(3,4);
               end
               robot.Loh=H;
           else
               robot.Loh=[1,0,0,0;
                0,1,0,robot.HandOffsetY;
                0,0,1,robot.HandOffsetZ;
                0,0,0,1];
           end           
           
           
           if exist('R','var')
                robot.ranges=R;
           end
           if exist('A','var')
                robot.A=A;
           else               
%                A=[ones(14,1),zeros(14,1)];
%                A(2,:)=[1,pi/2];
%                A(4,:)=[1,-pi/2];
%                
%                A(9,:)=[1,pi/2];
%                A(11,:)=[-1,-pi/2];
%                A(12,:)=[-1,0];
               robot.A=...
                   [1,0;            %left arm
                   1,0;
                   1,0;
                   1,0;
                   
                   1,0;
                   1,0;
                   1,0;
                   
                   1,0;             %right arm
                   1,0;
                   1,0;
                   1,0;
                   
                   1,0;
                   1,0;
                   1,0;];
           end
           if ~exist('model','var')
               robot.model='nao3.2';
           else
               robot.model=model;
           end
           if exist('S','var')
               robot.name=name;
           else
               robot.name=robot.model;
           end
           
           robot.B=zeros(size(robot.A));
           robot.B(:,1)=robot.A(:,1).^(-1);
           robot.B(:,2)=-(robot.A(:,2)).*(robot.A(:,1).^(-1));
           robot.ArmLength=robot.UpperArmLength+robot.LowerArmLength;   
           robot.ElbowOffsetY=0.0;
           robot.zerosAng=zeros(14,1);
           for i=1:14
               robot.zerosAng(i)=min(max(robot.zerosAng(i),robot.ranges(i,1)),robot.ranges(i,2));
           end
           
           %addpath('E:\Code\Research\Libraries\Matlab\naoqi-matlab-1.14\toolbox');
           %addpath('E:\Code\Research\Libraries\Matlab\naoqi-matlab-1.14');
           
           %todo remove dependence on Robotics Toolbox           
           
           robot.Lst=[  1,0,0,robot.ShoulderOffsetXL;
                        0,0,1,robot.ShoulderOffsetYL;
                        0,-1,0,robot.ShoulderOffsetZL;
                        0,0,0,1];
           robot.Lts=humanoid.Hinv(robot.Lst);  
           
           
           robot.Rst=[  1,0,0,robot.ShoulderOffsetXR;
                        0,0,1,robot.ShoulderOffsetYR;
                        0,-1,0,robot.ShoulderOffsetZR;
                        0,0,0,1];
           robot.Rts=humanoid.Hinv(robot.Rst);  
                      
            %convert from frame 2 in upper arm robot(upper end effector) to frame 2
            %(elbow) in full arm robot.
            robot.Lue_e=[0,1,0,0;  
                         0,0,1,0;
                         1,0,0,robot.UpperArmLength;
                         0,0,0,1];            
                      
            LL(1)=Link([0,0,0,pi/2]); 
            LL(2)=Link([0,0,robot.UpperArmLength,0]); 
            robot.leftUpper=SerialLink(LL,'name','LeftUpper','base',robot.Lst);
            robot.rightUpper=SerialLink(LL,'name','RightUpper','base',robot.Rst);

            NN(1)=Link([0,0,0,pi/2]);
            NN(2)=Link([0,0,0,pi/2]);
            NN(3)=Link([0,robot.UpperArmLength,0,-pi/2]);
            NN(4)=Link([0,0,robot.LowerArmLength,0]);
            robot.leftAll=SerialLink(NN,'name','Left','base',robot.Lst,'tool',robot.Loh);
            robot.rightAll=SerialLink(NN,'name','Right','base',robot.Rst,'tool',robot.Loh);
            
            
            
            %convert from frame 2 in upper arm robot(upper end effector) to frame 0
            %(elbow) in lower arm robot.
            if robot.isPanTiltShoulder
                robot.Lle_ue= [0,0,1,0;  
                               1,0,0,0;
                               0,1,0,0;
                               0,0,0,1];                            
            else
                robot.Lle_ue= [0,0,1,-robot.UpperArmLength;  
                               1,0,0,0;
                               0,1,0,0;
                               0,0,0,1];            
            end
            robot.Lue_le=humanoid.Hinv(robot.Lle_ue);
       end
   end
   methods
       % ang2pos runs forward kinematics. Input must be in NAO angles
       % format
       [ Tht,Tet] = iffkine(robot,ang,isRight)
       [ Aht,Aet] = iffkineP(robot,ang,isRight)
       % pos2ang solves inverse kinematics given the locations of elbow and
       % hand
       [ ang,singularityType,err] = ifikine(robot,Aet,Aht,isRight,ignoreHandTransformation)
       [ ang,singularityType,err] = ikine(robot,Tht,isRight,angHint)
       % retarget gets locations of elbow and hand in torso coordinates
       % given their locations in another humanoid skeleton
       [ Net, Nht ] = retarget(robot, Ast, Aet,Aht,isRight )
       
       %finds HeadYaw and HeadPitch to orient the head given a rotation
       %matrix
       [ang]=orientHead(robot, R);
       [ang]=pos2angH(robot, Ahn);
       
       % translation of angles
       [ ang ] = angFromStdAng(robot,dh, isRight )
       [ dh ] = angToStdAng( robot,ang, isRight )
       
       % range testing
       [ ok ] = isInRange( robot,ang, isRight)
       
       % convenient names for some functions
       function [ Tht,Tet] = ang2Transforms(robot,ang,isRight)
           if nargout>1
                [Tht,Tet]=iffkine(robot,ang,isRight);
           else
                Tht=iffkine(robot,ang,isRight);
           end
       end
       function [ Aht,Aet] = copypose(robot,ang,isRight)
           if nargout>1
                [Aht,Aet]=iffkineP(robot,ang,isRight);
           else
                Aht=iffkineP(robot,ang,isRight);
           end
       end
       function [ Aht,Aet] = ang2pos(robot,ang,isRight)
           if nargout>1
                [Aht,Aet]=iffkineP(robot,ang,isRight);
           else
                Aht=iffkineP(robot,ang,isRight);
           end
       end       
       function [ ang,singularityType,err] = pos2ang(robot,Aet,Aht,isRight,ignoreHandTransformation)
           [ ang,singularityType,err] = ifikine(robot,Aet,Aht,isRight,ignoreHandTransformation);
       end
   end
   methods(Access=protected,Static=true)
       [ theta ] = thetaFromThetaUL(thetaU,thetaL, isRight )
       [ thetaU,thetaL ] = thetaToThetaUL (theta, isRight )  
       [ H2 ] = Hinv( H )             
   end
   methods(Access=protected,Static=false)
       % these two functions convert between the internal full model of the
       % robot and the standard angles defined for the outside world. These
       % standard angles correspond to the NAO robot perfectly (this is why
       % A and B are 1 and 0 for the nao for all joints).
       % these functions must be called to convert a standard angle to the
       % correct theta values to be passed to the forward model of the
       % robot (e.g. in iffkine, iffkineP and ikine functions);
       function [ ang ] = sangFromDHTheta(robot,dh, isRight )
           if isvector(dh)
                N=1;
                nDof=numel(dh);
                ang=dh;
                ang(2)=dh(2)-pi/2;
                ang(4)=dh(4)+pi/2;
            else
                nDof=size(dh,2);
                N=size(dh,1);
                ang=dh;
                ang(:,2)=dh(:,2)-pi/2;
                ang(:,4)=dh(:,4)+pi/2;
            end
           
       end
       function [ dh ] = sangToDHTheta( robot,ang, isRight )
           if isvector(ang)
                N=1;
                nDof=numel(ang);
                dh=ang;
                dh(2)=ang(2)+pi/2;
                dh(4)=ang(4)-pi/2;
            else
                nDof=size(ang,2);
                N=size(ang,1);
                dh=ang;
                dh(:,2)=ang(:,2)+pi/2;
                dh(:,4)=ang(:,4)-pi/2;
           end                      
       end
   end
end

