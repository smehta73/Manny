classdef Manny
    %Class represents a humanoid robot with 6 joints in each leg and 5
    %joints in each arm.
    %   Detailed explanation goes here
    properties
       d_Torso
       d_hip_xLeft     
       d_hip_zLeft     
       d_hip_yLeft          
       d_knee_yLeft      
       d_foot_yLeft           
       d_foot_xLeft         
       d_shoulder_yLeft    
       d_shoulder_zLeft       
       d_elbow_xLeft       
       d_wrist_yLeft    
       d_hand_xLeft         
       d_foot_baseLeft
       d_foot_baseRight
       d_hip_xRight
       d_hip_zRight
       d_hip_yRight
       d_knee_yRight
       d_foot_yRight
       d_foot_xRight 
       d_shoulder_yRight
       d_shoulder_zRight
       d_elbow_xRight
       d_wrist_yRight
       d_hand_xRight
       d_footFrontEnd
       d_footBackEnd
       com_Torso
       com_hip_xLeft
       com_hip_zLeft
       com_hip_yLeft
       com_knee_yLeft
       com_foot_yLeft
       com_foot_xLeft
       com_shoulder_yLeft
       com_shoulder_zLeft
       com_elbow_xLeft
       com_wrist_yLeft
       com_hand_xLeft
       com_hip_xRight
       com_hip_zRight
       com_hip_yRight
       com_knee_yRight
       com_foot_yRight
       com_foot_xRight
       com_shoulder_yRight
       com_shoulder_zRight 
       com_elbow_xRight
       com_wrist_yRight
       com_hand_xRight 
       m_Torso 
       m_hip_x 
       m_hip_z 
       m_hip_y 
       m_knee_y 
       m_foot_y 
       m_foot_x  
       m_shoulder_y 
       m_shoulder_z 
       m_elbow_x 
       m_wrist_y
       m_hand_x 
       totalMass
       g_Torso
       g_hip_xLeft
       g_hip_zLeft
       g_hip_yLeft
       g_knee_yLeft
       g_foot_yLeft
       g_foot_xLeft
       g_shoulder_yLeft
       g_shoulder_zLeft
       g_elbow_xLeft
       g_wrist_yLeft 
       g_hand_xLeft
       g_foot_baseLeft
       g_foot_baseRight
       g_hip_xRight
       g_hip_zRight
       g_hip_yRight
       g_knee_yRight
       g_foot_yRight
       g_foot_xRight
       g_shoulder_yRight
       g_shoulder_zRight
       g_elbow_xRight
       g_wrist_yRight 
       g_hand_xRight
       g_footFrontEnd
       g_footBackEnd
       alpha
    end
    methods
       function object = Manny(alphas)
             object.d_Torso = [0;0;0];
             object.d_hip_xLeft = [-0.25;0.62;-2.19];     
             object.d_hip_zLeft = [-0.83;1.06;-2.29];      
             object.d_hip_yLeft = [0;0;-1.02];           
             object.d_knee_yLeft = [-0.57;0;-2.94];       
             object.d_foot_yLeft = [0.57;0;-2.95];            
             object.d_foot_xLeft = [0;0;0];          
             object.d_shoulder_yLeft = [-0.77;2.21;0.82];    
             object.d_shoulder_zLeft = [0.57;0.75;-0.82];        
             object.d_elbow_xLeft = [0;0.71;-0.91];         
             object.d_wrist_yLeft = [0.73;0; -2.28];    
             object.d_hand_xLeft = [0.70;-0.20;-0.12];          
             object.d_foot_baseLeft = [0; 0.33;-1.27]; 
             object.d_foot_baseRight = [0; -0.33;-1.27]; 
             object.d_hip_xRight = [-0.25;-0.62;-2.19];          
             object.d_hip_zRight = [-0.83;-1.06;-2.29];              
             object.d_hip_yRight = [0;0;-1.02];           
             object.d_knee_yRight = [-0.57;0;-2.94];            
             object.d_foot_yRight = [0.57;0;-2.95];           
             object.d_foot_xRight = [0;0;0];          
             object.d_shoulder_yRight = [-0.77;-2.21;0.82];    
             object.d_shoulder_zRight = [0.57;-0.75;-0.82];        
             object.d_elbow_xRight = [0;-0.71;-0.91];           
             object.d_wrist_yRight =  [0.73;0; -2.28];    
             object.d_hand_xRight = [0.70;0.20;-0.12];    
             object.d_footFrontEnd = [1.91; 0;0];
             object.d_footBackEnd = [-1.91;0;0];
             object.com_Torso = [-1.22;0;0.46];
             object.com_hip_xLeft = [-0.82;0.55;-0.74];
             object.com_hip_zLeft = [-0.01;-0.01;-0.43];
             object.com_hip_yLeft = [-0.01;-0.04;-1.73];
             object.com_knee_yLeft = [0.59;0.01;-1.35];
             object.com_foot_yLeft = [-0.78;-0.01;0.52];
             object.com_foot_xLeft = [0;0.24;-1.08];
             object.com_shoulder_yLeft = [-0.07;0.76;-0.03];
             object.com_shoulder_zLeft = [0;0.21;-0.36];
             object.com_elbow_xLeft = [0.12;0.03;-1.40];
             object.com_wrist_yLeft = [0.54; 0.02; -0.51];
             object.com_hand_xLeft = [1.20;-0.68;-0.49];
             object.com_hip_xRight = [-0.82;-0.55;-0.74];
             object.com_hip_zRight = [-0.01;0.01;-0.43];
             object.com_hip_yRight = [-0.01;0.04;-1.73];
             object.com_knee_yRight = [0.59;-0.01;-1.35];
             object.com_foot_yRight = [-0.78;0.01;0.52];
             object.com_foot_xRight = [0;-0.24;-1.08];
             object.com_shoulder_yRight = [-0.07;-0.76;-0.03];
             object.com_shoulder_zRight = [0;-0.21;-0.36];
             object.com_elbow_xRight = [0.12;-0.03;-1.40];
             object.com_wrist_yRight =  [0.54; -0.02; -0.51];
             object.com_hand_xRight = [1.20;0.68;-0.49];
             object.m_Torso = 0.243;
             object.m_hip_x = 0.123;
             object.m_hip_z = 0.0093;
             object.m_hip_y = 0.1242;
             object.m_knee_y = 0.0204;
             object.m_foot_y = 0.1255;
             object.m_foot_x = 0.0422;
             object.m_shoulder_y = 0.0574;
             object.m_shoulder_z = 0.0115;
             object.m_elbow_x = 0.1222;
             object.m_wrist_y = 0.0115;
             object.m_hand_x = 0.108;
             object.totalMass = 1.7534;
             object = object.setGTransforms(alphas);
        end
       function object = setGTransforms(object,alpha)
            object.alpha = alpha;
            Rx = @(alpha)[ 1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
            Ry = @(alpha)[cos(alpha), 0, sin(alpha); 0, 1, 0; -sin(alpha), 0, cos(alpha)];
            Rz = @(alpha)[cos(alpha), -sin(alpha), 0; sin(alpha), cos(alpha), 0; 0, 0, 1];
            object.g_Torso = SE3(object.d_Torso,eye(3));
            object.g_hip_xLeft = SE3(object.d_hip_xLeft,Rx(alpha(1)));
            object.g_hip_zLeft = SE3(object.d_hip_zLeft,Rz(alpha(2)));
            object.g_hip_yLeft = SE3(object.d_hip_yLeft,Ry(alpha(3)));
            object.g_knee_yLeft = SE3(object.d_knee_yLeft,Ry(alpha(4)));
            object.g_foot_yLeft = SE3(object.d_foot_yLeft,Ry(alpha(5)));
            object.g_foot_xLeft = SE3(object.d_foot_xLeft,Rx(alpha(6)));
            object.g_shoulder_yLeft = SE3(object.d_shoulder_yLeft,Ry(alpha(7)));
            object.g_shoulder_zLeft = SE3(object.d_shoulder_zLeft,Ry(alpha(8)));
            object.g_elbow_xLeft = SE3(object.d_elbow_xLeft,Rx(alpha(9)));
            object.g_wrist_yLeft = SE3(object.d_wrist_yLeft,Ry(alpha(10)));
            object.g_hand_xLeft = SE3(object.d_hand_xLeft,Rx(alpha(11)));
            object.g_foot_baseLeft = SE3(object.d_foot_baseLeft,eye(3));
            object.g_foot_baseRight = SE3(object.d_foot_baseRight,eye(3));
            object.g_hip_xRight = SE3(object.d_hip_xRight,Rx(alpha(12)));
            object.g_hip_zRight = SE3(object.d_hip_zRight,Rz(alpha(13)));
            object.g_hip_yRight = SE3(object.d_hip_yRight,Ry(alpha(14)));
            object.g_knee_yRight = SE3(object.d_knee_yRight,Ry(alpha(15)));
            object.g_foot_yRight = SE3(object.d_foot_yRight,Ry(alpha(16)));
            object.g_foot_xRight = SE3(object.d_foot_xRight,Rx(alpha(17)));
            object.g_shoulder_yRight = SE3(object.d_shoulder_yRight,Ry(alpha(18)));
            object.g_shoulder_zRight = SE3(object.d_shoulder_zRight,Ry(alpha(19)));
            object.g_elbow_xRight = SE3(object.d_elbow_xRight,Rx(alpha(20)));
            object.g_wrist_yRight = SE3(object.d_wrist_yRight, Ry(alpha(21)));
            object.g_hand_xRight = SE3(object.d_hand_xRight,Rx(alpha(22)));
            object.g_footFrontEnd= SE3(object.d_footFrontEnd,eye(3));
            object.g_footBackEnd = SE3(object.d_footBackEnd, eye(3));
        end            
       function [xCoM,yCoM,zCoM] = CoM(object,frameOfReference)
            comLeftArmPart1 = object.m_shoulder_y * (object.g_Torso * object.g_shoulder_yLeft .* [object.com_shoulder_yLeft ;1]);
            comLeftArmPart2 = object.m_shoulder_z * (object.g_Torso * object.g_shoulder_yLeft * object.g_shoulder_zLeft .*[object.com_shoulder_zLeft;1]) ;
            comLeftArmPart3 = object.m_elbow_x * (object.g_Torso * object.g_shoulder_yLeft * object.g_shoulder_zLeft * object.g_elbow_xLeft .*[object.com_elbow_xLeft;1]);
            comLeftArmPart4 = object.m_wrist_y * (object.g_Torso * object.g_shoulder_yLeft * object.g_shoulder_zLeft * object.g_elbow_xLeft * object.g_wrist_yLeft .* [object.com_wrist_yLeft;1]);
            comLeftArmPart5 = object.m_hand_x * (object.g_Torso * object.g_shoulder_yLeft * object.g_shoulder_zLeft * object.g_elbow_xLeft * object.g_wrist_yLeft * object.g_hand_xLeft .* [object.com_hand_xLeft;1]);
            comLeftArm = (comLeftArmPart1 + comLeftArmPart2 + comLeftArmPart3 + comLeftArmPart4 + comLeftArmPart5)/object.totalMass;
            comLeftLegPart1 = object.m_hip_x * (object.g_Torso * object.g_hip_xLeft .* [object.com_hip_xLeft;1]);
            comLeftLegPart2 = object.m_hip_z * (object.g_Torso * object.g_hip_xLeft * object.g_hip_zLeft .*[object.com_hip_zLeft;1]);
            comLeftLegPart3 = object.m_hip_y * (object.g_Torso * object.g_hip_xLeft * object.g_hip_zLeft * object.g_hip_yLeft .* [object.com_hip_yLeft;1]);
            comLeftLegPart4 = object.m_knee_y * (object.g_Torso * object.g_hip_xLeft * object.g_hip_zLeft * object.g_hip_yLeft * object.g_knee_yLeft .* [object.com_knee_yLeft;1]);
            comLeftLegPart5 = object.m_foot_y * (object.g_Torso * object.g_hip_xLeft * object.g_hip_zLeft * object.g_hip_yLeft * object.g_knee_yLeft * object.g_foot_yLeft .* [object.com_foot_yLeft;1]);
            comLeftLegPart6 = object.m_foot_x * (object.g_Torso * object.g_hip_xLeft * object.g_hip_zLeft* object.g_hip_yLeft * object.g_knee_yLeft * object.g_foot_yLeft * object.g_foot_xLeft .* [object.com_foot_xLeft;1]);
            comLeftLeg = (comLeftLegPart1 + comLeftLegPart2 + comLeftLegPart3 + comLeftLegPart4 + comLeftLegPart5 + comLeftLegPart6)/object.totalMass; 
            comRightArmPart1 = object.m_shoulder_y * (object.g_Torso * object.g_shoulder_yRight .* [object.com_shoulder_yRight;1]); 
            comRightArmPart2 = object.m_shoulder_z * (object.g_Torso * object.g_shoulder_yRight * object.g_shoulder_zRight .*[object.com_shoulder_zRight;1]) ;
            comRightArmPart3 = object.m_elbow_x * (object.g_Torso * object.g_shoulder_yRight * object.g_shoulder_zRight * object.g_elbow_xRight .*[object.com_elbow_xRight;1]);
            comRightArmPart4 = object.m_wrist_y * (object.g_Torso * object.g_shoulder_yRight * object.g_shoulder_zRight * object.g_elbow_xRight * object.g_wrist_yRight .* [object.com_wrist_yRight;1]);
            comRightArmPart5 = object.m_hand_x * (object.g_Torso * object.g_shoulder_yRight * object.g_shoulder_zRight * object.g_elbow_xRight * object.g_wrist_yRight * object.g_hand_xRight .* [object.com_hand_xRight;1]);
            comRightArm = (comRightArmPart1 + comRightArmPart2 + comRightArmPart3 + comRightArmPart4 + comRightArmPart5)/object.totalMass;
            comRightLegPart1 = object.m_hip_x * (object.g_Torso * object.g_hip_xRight .* [object.com_hip_xRight;1]);
            comRightLegPart2 = object.m_hip_z * (object.g_Torso * object.g_hip_xRight * object.g_hip_zRight .*[object.com_hip_zRight;1]);
            comRightLegPart3 = object.m_hip_y * (object.g_Torso * object.g_hip_xRight * object.g_hip_zRight * object.g_hip_yRight .* [object.com_hip_yRight;1]);
            comRightLegPart4 = object.m_knee_y * (object.g_Torso * object.g_hip_xRight * object.g_hip_zRight * object.g_hip_yRight * object.g_knee_yRight .* [object.com_knee_yRight;1]);
            comRightLegPart5 = object.m_foot_y * (object.g_Torso * object.g_hip_xRight * object.g_hip_zRight * object.g_hip_yRight * object.g_knee_yRight * object.g_foot_yRight .* [object.com_foot_yRight;1]);
            comRightLegPart6 = object.m_foot_x * (object.g_Torso * object.g_hip_xRight * object.g_hip_zRight * object.g_hip_yRight * object.g_knee_yRight * object.g_foot_yRight * object.g_foot_xRight .* [object.com_foot_xRight;1]);
            comRightLeg = (comRightLegPart1 + comRightLegPart2 + comRightLegPart3 + comRightLegPart4 + comRightLegPart5 + comRightLegPart6)/object.totalMass;
            comTorso = object.m_Torso * (object.g_Torso .* [object.com_Torso;1])/object.totalMass;
            centerOfMassFromTorso =(comLeftArm + comRightArm + comLeftLeg + comRightLeg + comTorso);
            centerOfMassFromRightLeg= inv(object.g_foot_baseRight)* inv(object.g_foot_xRight) * inv(object.g_foot_yRight) * inv(object.g_knee_yRight) * inv(object.g_hip_yRight) * inv(object.g_hip_zRight) * inv(object.g_hip_xRight) * inv(object.g_Torso) .* centerOfMassFromTorso;
            centerOfMassFromLeftLeg = inv(object.g_foot_baseLeft)* inv(object.g_foot_xLeft)  * inv(object.g_foot_yLeft)  * inv(object.g_knee_yLeft)  * inv(object.g_hip_yLeft)  * inv(object.g_hip_zLeft)  * inv(object.g_hip_xLeft)  * inv(object.g_Torso) .* centerOfMassFromTorso;
            if (frameOfReference == 0)
                xCoM = centerOfMassFromTorso(1);
                yCoM = centerOfMassFromTorso(2);
                zCoM = centerOfMassFromTorso(3);
            elseif (frameOfReference == 2)
                xCoM = centerOfMassFromRightLeg(1);
                yCoM = centerOfMassFromRightLeg(2);
                zCoM = centerOfMassFromRightLeg(3);
            elseif (frameOfReference == 1)
                xCoM = centerOfMassFromLeftLeg(1);
                yCoM = centerOfMassFromLeftLeg(2);
                zCoM = centerOfMassFromLeftLeg(3);
            end
       end
       function [positionOfRightFootBase, positionOfLeftFootBase, LeftFootFront, LeftFootBack, RightFootFront, RightFootBack] = footPositions(object)
                gT_LFB = object.g_Torso * object.g_hip_xLeft * object.g_hip_zLeft * object.g_hip_yLeft * object.g_knee_yLeft * object.g_foot_yLeft * object.g_foot_xLeft * object.g_foot_baseLeft;
                gT_RFB = object.g_Torso * object.g_hip_xRight * object.g_hip_zRight * object.g_hip_yRight * object.g_knee_yRight * object.g_foot_yRight * object.g_foot_xRight * object.g_foot_baseRight;
                gLFB_RFB = inv(gT_LFB) * gT_RFB;
                gRFB_LFB = inv(gT_RFB) * gT_LFB;
                gLFB_RFFront = inv(gT_LFB) *gT_RFB * object.g_footFrontEnd;
                gLFB_RFBack = inv(gT_LFB) *gT_RFB * object.g_footBackEnd;
                gRFB_LFFront = inv(gT_RFB) * gT_LFB * object.g_footFrontEnd;
                gRFB_LFBack = inv(gT_RFB) * gT_LFB * object.g_footBackEnd;
                positionOfRightFootBase = getTranslation(gLFB_RFB);
%                 rotationOfRightFootBase = getRotation(gLFB_RFB);
%                 positionOfLeftFootBase =  -rotationOfRightFootBase.' * positionOfRightFootBase; 

                positionOfLeftFootBase = getTranslation(gRFB_LFB);
                LeftFootFront = getTranslation(gRFB_LFFront);
                LeftFootBack = getTranslation(gRFB_LFBack);
                RightFootFront = getTranslation(gLFB_RFFront);
                RightFootBack = getTranslation(gLFB_RFBack);          
       end
       function supportPolygon(object,howManyFeetOnGround,whichFootOnGround)
        PosRightFootwrtTorso = object.g_Torso* object.g_hip_xRight * object.g_hip_zRight * object.g_hip_yRight * object.g_knee_yRight * object.g_foot_yRight * object.g_foot_xRight * object.g_foot_baseRight;
        PosRightFoot =  inv(PosRightFootwrtTorso) .* getTranslation(PosRightFootwrtTorso);
        PosLeftFootwrtTorso =  object.g_Torso* object.g_hip_xLeft * object.g_hip_zLeft * object.g_hip_yLeft * object.g_knee_yLeft * object.g_foot_yLeft * object.g_foot_xLeft * object.g_foot_baseLeft;
        PosLeftFoot = inv(PosLeftFootwrtTorso) .* getTranslation(PosLeftFootwrtTorso);
        if (howManyFeetOnGround == 1) 
            if (whichFootOnGround == 2)%right leg 
                x= PosRightFoot(1)+[1.91,1.91,1.91,1.69,0,-1.69,-1.91,-1.91,-1.91,-1.69,0,1.69];
                y=PosRightFoot(2)+[-.73,0,.73,1.1,1.1,1.1,.73,0,-.73,-1.1,-1.1,-1.1];
                z =PosRightFoot(3)*ones(1,12);
                K = convhull(x,y);
                plot3(x(K), y(K),z(K),'r-',x,y,z,'b*')
                hold on
            elseif(whichFootOnGround == 1)%left leg
                x= PosLeftFoot(1)+[1.91,1.91,1.91,1.69,0,-1.69,-1.91,-1.91,-1.91,-1.69,0,1.69];
                y=PosLeftFoot(2)+[-.73,0,.73,1.1,1.1,1.1,.73,0,-.73,-1.1,-1.1,-1.1];
                z =PosLeftFoot(3)*ones(1,12);
                K = convhull(x,y);
                plot3(x(K), y(K),z(K),'r-',x,y,z,'b*')
                hold on
            end
        end
        if (howManyFeetOnGround == 2) 
            xPartOne = PosLeftFoot(1)+[1.91,1.91,1.91,1.69,0,-1.69,-1.91,-1.91,-1.91,-1.69,0,1.69];
            xPartTwo = PosRightFoot(1)+[1.91,1.91,1.91,1.69,0,-1.69,-1.91,-1.91,-1.91,-1.69,0,1.69];
            yPartOne = PosLeftFoot(2)+[-.73,0,.73,1.1,1.1,1.1,.73,0,-.73,-1.1,-1.1,-1.1];
            yPartTwo = PosRightFoot(2)+[-.73,0,.73,1.1,1.1,1.1,.73,0,-.73,-1.1,-1.1,-1.1];
            x = [xPartOne, xPartTwo];
            y = [yPartOne, yPartTwo];
            z =PosRightFoot(3)*ones(1,24);
            K = convhull(x,y);
            plot3(x(K),y(K),z(K),'r-',x,y,z,'b*')
            hold on
        end
       end
       function visualization(object,whichFootisSupport)
          motorlength= 5/2.54;
          motorwidth= 3.2/2.54;
          motorheight= 4/2.54;
          Torsolength = 10.5/2.54;
          Torsowidth = 10.2/2.54;
          Torsoheight = 6.35/2.54;
          Footbaselength = 2.2;
          Footbasewidth = 3.82;
          gray = [56, 56, 56]/256;
          red  = [1, 0.1, 0.1];
          blue = [0.1, 0.1, 0.8];
          green = [0.1, 1, 0.1];
          gLeftleg1 = object.g_Torso * object.g_hip_xLeft* SE3([-0.81;0.57;0.01],eye(3));
          gRightleg1 = object.g_Torso * object.g_hip_xRight* SE3([-0.81;-0.57;0.01],eye(3));
          gLeftleg2 = object.g_Torso * object.g_hip_xLeft* SE3([-0.81;0.57;-1.5],eye(3));
          gRightleg2 = object.g_Torso * object.g_hip_xRight* SE3([-0.81;-0.57;-1.5],eye(3));
          gLeftleg3 = object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft *object.g_hip_yLeft* SE3([-0.01; -0.03;-0.5],eye(3));
          gRightleg3 = object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* SE3([-0.01;0.03;-0.5],eye(3));
          gLeftleg4 = object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft*object.g_knee_yLeft * SE3([0.49; -0.03;0],eye(3));
          gRightleg4 = object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * SE3([0.49; 0.03;0],eye(3));
          gLeftleg5 = object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* SE3([-0.01; -0.03;0.5],eye(3));
          gRightleg5 = object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight * SE3([-0.01; 0.03;0.50],eye(3));
          gLeftleg6 = object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* SE3([-1.69; 0.01;0.49],eye(3));
          gRightleg6 = object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight * SE3([-1.69;-0.01;0.49],eye(3));
          gLeftleg7 = object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* object.g_foot_xLeft* object.g_foot_baseLeft;
          gRightleg7 = object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight * object.g_knee_yRight *object.g_foot_yRight * object.g_foot_xRight* object.g_foot_baseRight;
          gRightShoulder1= object.g_Torso* object.g_shoulder_yRight;
          gLeftShoulder1=  object.g_Torso* object.g_shoulder_yLeft;
          gRightShoulder2= object.g_Torso* object.g_shoulder_yRight * object.g_shoulder_zRight * object.g_elbow_xRight *SE3([0.04; -0.02;-0.5],eye(3));
          gLeftShoulder2= object.g_Torso* object.g_shoulder_yLeft * object.g_shoulder_zLeft *object.g_elbow_xLeft * SE3([0.04; 0.02;-0.5],eye(3));
          gRightShoulder3 = object.g_Torso* object.g_shoulder_yRight * object.g_shoulder_zRight * object.g_elbow_xRight* object.g_wrist_yRight * SE3([-0.51; -0.06; 0],eye(3)); 
          gLeftShoulder3 = object.g_Torso* object.g_shoulder_yLeft * object.g_shoulder_zLeft *object.g_elbow_xLeft * object.g_wrist_yLeft *SE3([-0.51; 0.06; 0],eye(3));
          if(whichFootisSupport == 1)%with respect to left leg
              gFB_T = inv(gLeftleg7);
              [a, b,c] = object.CoM(1);
          elseif(whichFootisSupport == 2)%with respect to right leg
              gFB_T = inv(gRightleg7);
              [a, b,c] = object.CoM(2);
          end
          clf;
          shBlock(gFB_T * object.g_Torso, [Torsolength Torsowidth Torsoheight], gray);
          hold on;
          shBlock(gFB_T * gLeftleg1, [motorheight,motorlength, motorwidth], blue);
          shBlock(gFB_T * gRightleg1, [motorheight, motorlength, motorwidth], blue);
          shBlock(gFB_T * gLeftleg2, [motorheight, motorlength, motorwidth], red);
          shBlock(gFB_T * gRightleg2, [motorheight, motorlength, motorwidth],green);
          shBlock(gFB_T * gLeftleg3,[motorheight, motorwidth, motorlength] , red);
          shBlock(gFB_T * gRightleg3, [motorheight, motorwidth, motorlength], green);
          shBlock(gFB_T * gLeftleg4,[motorlength, motorwidth, motorheight], red);
          shBlock(gFB_T * gRightleg4, [motorlength, motorwidth, motorheight],green);
          shBlock(gFB_T * gLeftleg5,[motorheight, motorwidth, motorlength], red);
          shBlock(gFB_T * gRightleg5,[motorheight, motorwidth, motorlength],green);
          shBlock(gFB_T * gLeftleg6,[motorheight, motorwidth, motorlength], red);
          shBlock(gFB_T * gRightleg6,[motorheight, motorwidth, motorlength],green);
          shBlock(gFB_T *gLeftleg7 ,[Footbasewidth, Footbaselength,0], gray);
          shBlock(gFB_T *gRightleg7,[Footbasewidth,Footbaselength ,0], gray);
          shBlock(gFB_T *gRightShoulder1, [motorwidth, motorlength, motorheight], blue);
          shBlock(gFB_T *gLeftShoulder1, [motorwidth, motorlength, motorheight], blue);
          shBlock(gFB_T *gRightShoulder2,[motorheight, motorwidth, motorlength] , blue);
          shBlock(gFB_T *gLeftShoulder2, [motorheight, motorwidth, motorlength], blue);
          shBlock(gFB_T * gRightShoulder3,[motorlength,motorwidth, motorheight], blue);
          shBlock(gFB_T * gLeftShoulder3 , [motorlength,motorwidth,motorheight], blue );
          hold on
          object.supportPolygon(1,whichFootisSupport)
          plot3(a, b, c,' m*')
          axis equal
          axis ([-8, 8, -10, 10, -8, 24]); 
          view([-238 9]);
          rotate3d on;
          xlabel('x');
          ylabel('y');
          zlabel('z');
       end
       function jacobian = computeJacobianOfLeg(object,whichLeg)
           if (whichLeg == 1) 
               g1 = object.g_hip_xLeft;  
               g2 = object.g_hip_zLeft;  
               g3 = object.g_hip_yLeft; 
               g4 = object.g_knee_yLeft;
               g5 = object.g_foot_yLeft ;
               g6 = object.g_foot_xLeft; 
           elseif(whichLeg == 2)
               g1 = object.g_hip_xRight;  
               g2 = object.g_hip_zRight;  
               g3 = object.g_hip_yRight; 
               g4 = object.g_knee_yRight;
               g5 = object.g_foot_yRight;
               g6 = object.g_foot_xRight; 
           end
           g1to1 = g1;
           r1to1 = getRotation(g1to1);
           d1to1 = getTranslation(g1to1);
           g1to2 = g1 * g2;
           r1to2 = getRotation(g1to2);
           d1to2 = getTranslation(g1to2);
           g1to3 = g1 * g2 * g3;
           r1to3 = getRotation(g1to3);
           d1to3 = getTranslation(g1to3);
           g1to4 = g1 * g2 * g3 * g4;
           r1to4 = getRotation(g1to4);
           d1to4 = getTranslation(g1to4);
           g1to5 = g1 * g2 * g3 * g4 * g5;
           r1to5 = getRotation(g1to5);
           d1to5 = getTranslation(g1to5);
           g1to6 = g1 * g2 * g3 * g4 * g5 * g6;
           r1to6 = getRotation(g1to6);
           d1to6 = getTranslation(g1to6);
           linearJacobianColumn1 = cross(r1to1(:,1),(d1to6-d1to1));
           linearJacobianColumn2 = cross(r1to2(:,3),(d1to6-d1to2));
           linearJacobianColumn3 = cross(r1to3(:,2),(d1to6-d1to3));
           linearJacobianColumn4 = cross(r1to4(:,2),(d1to6-d1to4));
           linearJacobianColumn5 = cross(r1to5(:,2),(d1to6-d1to5));
           linearJacobianColumn6 = cross(r1to6(:,1),(d1to6-d1to6));
           linearJacobian = [linearJacobianColumn1 linearJacobianColumn2 linearJacobianColumn3 linearJacobianColumn4 linearJacobianColumn5 linearJacobianColumn6]; 
           angularJacobian = [r1to1(:,1),r1to2(:,3),r1to3(:,2),r1to4(:,2),r1to5(:,2),r1to6(:,1)];
           jacobian = [linearJacobian;angularJacobian];
       end
       function object = kickLeg(object,whichLeg)
          if (whichLeg == 1)
                finalG = (object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* object.g_foot_xLeft* object.g_foot_baseLeft*SE3([1;0;1], eye(3)));
                finalPos = getTranslation(finalG);
                currentG = (object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* object.g_foot_xLeft* object.g_foot_baseLeft);
                currentPos = getTranslation(currentG);
          elseif(whichLeg == 2)
                finalG = (object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight* object.g_foot_xRight* object.g_foot_baseRight*SE3([1; 0;1], eye(3)));
                finalPos = getTranslation(finalG);
                currentG = (object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight* object.g_foot_xRight* object.g_foot_baseRight); 
                currentPos = getTranslation(currentG);
          end
           distance = finalPos - currentPos;
           t=7;
           while (sqrt((distance(1)^2)+ (distance(2)^2)+(distance(3)^2)) > 0.015 && t>0)
               endEffectorLinearVelocity = distance/t;
               [jacobian] = object.computeJacobianOfLeg(whichLeg);
               weight = [1,0,0,0,0,0; 0,1,0,0,0,0; 0,0,1,0,0,0; 0,0,0,.1,0,0; 0,0,0,0,1,0; 0,0,0,0,0,1];
               linearJacobian = jacobian(1:3,:);
               alphaVelocity = inv((linearJacobian' * linearJacobian) - weight) * linearJacobian' * endEffectorLinearVelocity;
               if(whichLeg == 1)
                     object.visualization(2);
                     object.alpha(1:6) = object.alpha(1:6) + transpose(alphaVelocity);
                     object = object.setGTransforms(object.alpha);
                     currentG = (object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* object.g_foot_xLeft* object.g_foot_baseLeft);
                     currentPos = getTranslation(currentG);
               elseif(whichLeg == 2)
                     object.visualization(1);
                     object.alpha(12:17) = object.alpha(12:17) + transpose(alphaVelocity);
                     object = object.setGTransforms(object.alpha);
                     currentG = (object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight* object.g_foot_xRight* object.g_foot_baseRight); 
                     currentPos = getTranslation(currentG);
               end
               distance = (finalPos- currentPos)
               t=t-0.5;
               pause(2)
           end           
       end
       function object = kickLegProper(object,whichLeg)
           if (whichLeg == 1)
                finalG = (object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* object.g_foot_xLeft* object.g_foot_baseLeft*SE3([1;0;1], eye(3)));
                finalPos = getTranslation(finalG);
                currentG = (object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* object.g_foot_xLeft* object.g_foot_baseLeft);
                currentPos = getTranslation(currentG);
           elseif(whichLeg == 2)
                finalG = (object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight* object.g_foot_xRight* object.g_foot_baseRight*SE3([1; 0;1], eye(3)));
                finalPos = getTranslation(finalG);
                currentG = (object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight* object.g_foot_xRight* object.g_foot_baseRight); 
                currentPos = getTranslation(currentG);
           end
           distance = finalPos - currentPos;
           t=7;
           while (sqrt((distance(1)^2)+ (distance(2)^2)+(distance(3)^2)) > 0.015 && t>0)
               differenceG = inv(currentG) * finalG;
               differenceR = getRotation(differenceG);
               rotationAxisAndAngle = vrrotmat2vec(differenceR);
               rotationAngle = rotationAxisAndAngle(4);
               rotationAxis = transpose(rotationAxisAndAngle(1:3));
               endEffectorAngularVelocity = (rotationAxis*rotationAngle)/t;
               endEffectorLinearVelocity = distance/t;
               jacobian = object.computeJacobianOfLeg(whichLeg);
               endEffectorVelocity = [endEffectorLinearVelocity;endEffectorAngularVelocity];
               pseudoJacobianInverse = pinv(jacobian);
               alphaVelocity = pseudoJacobianInverse * endEffectorVelocity;
%                weight = [1,0,0,0,0,0; 0,1,0,0,0,0; 0,0,1,0,0,0; 0,0,0,.1,0,0; 0,0,0,0,1,0; 0,0,0,0,0,1];
%                alphaVelocity = inv((pseudoJacobianInverse' * pseudoJacobianInverse) - weight) * pseudoJacobianInverse' * endEffectorVelocity;
               if(t == 7)
                   linearJacobian = jacobian(1:3,:); 
                   weight = [1,0,0,0,0,0; 0,1,0,0,0,0; 0,0,1,0,0,0; 0,0,0,.1,0,0; 0,0,0,0,1,0; 0,0,0,0,0,1];
                   alphaVelocity = inv((linearJacobian' * linearJacobian) - weight) * linearJacobian' * endEffectorLinearVelocity;
               end
               if(whichLeg == 1)
                     object.visualization(2);
                     object.alpha(1:6) = object.alpha(1:6) + transpose(alphaVelocity);
                     object = object.setGTransforms(object.alpha);
                     currentG = (object.g_Torso * object.g_hip_xLeft* object.g_hip_zLeft * object.g_hip_yLeft* object.g_knee_yLeft * object.g_foot_yLeft* object.g_foot_xLeft* object.g_foot_baseLeft);
                     currentPos = getTranslation(currentG);
               elseif(whichLeg == 2)
                     object.visualization(1);
                     object.alpha(12:17) = object.alpha(12:17) + transpose(alphaVelocity);
                     object = object.setGTransforms(object.alpha);
                     currentG = (object.g_Torso * object.g_hip_xRight* object.g_hip_zRight * object.g_hip_yRight* object.g_knee_yRight * object.g_foot_yRight* object.g_foot_xRight* object.g_foot_baseRight); 
                     currentPos = getTranslation(currentG);
               end
               distance = (finalPos- currentPos);
               t=t-0.5;
               pause(2)
           end           
       end  
    end
       
end
