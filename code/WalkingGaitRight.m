MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );
close all
clear all
dxl_io = Dynamixel_IO;
dxl_io.load_library();
dxl_io.connect(0, 1); 
load('IPPart1Alphas.mat');
load('IPPart2Alphas.mat');
load('IPPart3Alphas.mat');

motor_ids = [10,12,14,16,18,9,11,13,15,17];
a = [IPPart1Alphas IPPart2Alphas IPPart3Alphas IPPart1Alphas IPPart2Alphas IPPart3Alphas];

load('Motor_Biases.mat');
Motor_Biases = -1* motor_bias;
% motorBias =[Motor_Biases(2:2:10); Motor_Biases(1:2:9)];
%  motorBias =[Motor_Biases(2:2:10); Motor_Biases(2:2:10)];
motorBias = zeros(10,1);
% motorBias(2,1) = 3;
% motorBias(1,1) = 10;
motor_ids_initial =[ 1:5,9,10, 17:22];
bias = [zeros(1,5), Motor_Biases(1:2)',Motor_Biases(9:12)',0,0]'  ;
dxl_io.set_compliance_margin(motor_ids_initial, zeros(13,1), 'cw');
dxl_io.set_motor_pos_speed(motor_ids_initial,bias,zeros(length(motor_ids_initial),1), ones(length(motor_ids_initial),1)*pi/20);
pause(1);
dxl_io.set_motor_pos_speed(6,0,-pi/2,pi/6);
dxl_io.set_motor_pos_speed(7,0,pi/4,pi/6);
pause(0.1);
dxl_io.set_motor_pos_speed(8,0,-pi/4,pi/6);
pause(0.1)

downSample = 2;
timeStep = 0.1;
pauseTime = 0.13;
kI = zeros(10,6);
integral = zeros(10,1);
start = 1;
final = 100;
kI(1,1) = 0.1;
kI(2,1) = 0;
kI(3,1) = 0;
kI(4,1) = 0;
kI(5,1) = 0;
kI(6,1) = 0.1;
kI(7,1) = 0.1;
kI(8,1) = 0.1;
kI(9,1) = 0.1;
kI(10,1) = 0;
initialPositions = [-a(1,start);a(2,start); a(3,start) ; a(4,start);a(5,start); -a(6,start);-a(7,start); a(8,start); -a(9,start);a(10,start)];
initialPositions = -circshift(initialPositions,5);
dxl_io.set_motor_pos_speed(motor_ids, motorBias,initialPositions, ones(length(motor_ids),1)*pi/6);
percentError = @(x,y) 100 *((y-x)./y) ;
feedbackI=zeros(10,1);
pause(5);
currentMotorPosition = dxl_io.read_present_pos_vel(motor_ids', 0, 'pos');
errorDegrees = [];
i=1;
phase = 1;
ind = 1;
for i = start + downSample-1:downSample:final
    dest_pos(1) = -a(1,i);
    dest_pos(2) = a(2,i) ;
    dest_pos(3) = a(3,i);
    dest_pos(4) = a(4,i) ;
    dest_pos(5) = a(5,i) ;
    dest_pos(6) = -a(6,i);
    dest_pos(7) = -a(7,i) ;
    dest_pos(8) = a(8,i);
    dest_pos(9) = -a(9,i) ;
    dest_pos(10) = a(10,i);
    dest_speed(1) = max(abs((a(1,i) - a(1,i-downSample + 1)) /timeStep),0.1745);  
    dest_speed(2) = max(abs((a(2,i) - a(2,i-downSample + 1)) /timeStep), 0.1745); 
    dest_speed(3) = max(abs((a(3,i) - a(3,i-downSample + 1)) /timeStep), 0.1745); 
    dest_speed(4) = max(abs((a(4,i) - a(4,i-downSample + 1)) /timeStep), 0.1745); 
    dest_speed(5) = max(abs((a(5,i) - a(5,i-downSample + 1)) /timeStep), 0.1745);    
    dest_speed(6) = max(abs((a(6,i) - a(6,i-downSample + 1)) /timeStep), 0.1745);   
    dest_speed(7) = max(abs((a(7,i) - a(7,i-downSample + 1)) /timeStep),0.1745);  
    dest_speed(8) = max(abs((a(8,i) - a(8,i-downSample + 1)) /timeStep),0.1745); 
    dest_speed(9) = max(abs((a(9,i) - a(9,i-downSample + 1)) /timeStep),0.1745);   
    dest_speed(10) = max(abs((a(10,i) - a(10,i-downSample + 1)) /timeStep),0.1745);
    if (i < 301)
        dest_pos = -circshift(dest_pos,5);
    end
%     dxl_io.set_motor_pos_speed(motor_ids,-motorBias,dest_pos' + feedbackI, dest_speed');
      for motor = [1:10]
            dxl_io.set_motor_pos_speed(motor_ids(motor),-motorBias(motor),dest_pos(motor)' + feedbackI(motor), dest_speed(motor)');
      end
    if(i == 100)
           finalComandValues = dest_pos' + feedbackI;
           kI(:,2) = kI(:,1); 
           kI(10,2) = 0.1;
           phase = 2;
    end
    
    if(i == 200)
        kI(:,3) = kI(:,2);
         kI(10,3) = 0.05;
        phase = 3;
    end
    if(i == 300)
        kI(1,1) = 0.1;
        kI(2,1) = 0.1;
        kI(3,1) = 0.1;
        kI(4,1) = 0.1;
        kI(5,1) = 0;
        kI(6,1) = 0.1;
        kI(7,1) = 0;
        kI(8,1) = 0;
        kI(9,1) = 0;
        kI(10,1) = 0;
        phase = 4;
    end
    pause(pauseTime);
    currentMotorPosition = dxl_io.read_present_pos_vel(motor_ids', 0, 'pos');
    goalMotorPosition = dest_pos';
    for y=1:10
            if abs(goalMotorPosition(y) - currentMotorPosition(y)) < deg2rad(50)
                error(y,1) = goalMotorPosition(y,1) - currentMotorPosition(y,1);
            else
               if(ind == 1)
                    error(y,1) = 0;
               else 
                   error(y,1) = pastError(y,1);
               end
                disp(['bad motor data read: ' num2str(motor_ids(y)) ' sample number: ' num2str(ind)])       
            end
        end
        pastError = error;
        integral = integral + error;
        feedbackI =kI(:,phase) .* integral ;
        errorDegrees = [errorDegrees error];
        ind = ind + 1;
end 
errorDegrees = rad2deg(errorDegrees);
figure;
plot(errorDegrees(1,:))
title(['Swing Hip Roll kI = ',num2str(kI(1,1)),',' , num2str(kI(1,2)),',',num2str(kI(1,3))]);
figure;
plot(errorDegrees(2,:))
title(['Swing Hip Pitch kI = ',num2str(kI(2,1)),',',num2str(kI(2,2)),',',num2str(kI(2,3))]);
figure;
plot(errorDegrees(3,:))
title(['Swing Knee Pitch kI = ',num2str(kI(3,1)),',',num2str(kI(3,2)),',',num2str(kI(3,3))]);
figure;
plot(errorDegrees(4,:))
title(['Swing Ankle Pitch kI = ',num2str(kI(4,1)),',',num2str(kI(4,2)),',',num2str(kI(4,3))]);
figure;
plot(errorDegrees(5,:))
title(['Swing Ankle Roll kI = ',num2str(kI(5,1)),',', num2str(kI(5,2)),',',num2str(kI(5,3))]);
figure;
plot(errorDegrees(6,:))
title(['Stance Hip Roll kI = ',num2str(kI(6,1)),',',num2str(kI(6,2)),',',num2str(kI(6,3))]);
figure;
plot(errorDegrees(7,:))
title(['Stance Hip Pitch kI = ',num2str(kI(7,1)),',',num2str(kI(1,2)),',',num2str(kI(7,3))]);
figure;
plot(errorDegrees(8,:))
title(['Stance Knee Pitch kI = ',num2str(kI(8,1)),',' ,num2str(kI(8,2)),',',num2str(kI(8,3))]);
figure;
plot(errorDegrees(9,:))
title(['Stance Ankle Pitch kI = ',num2str(kI(9,1)),',' , num2str(kI(9,2)),',',num2str(kI(9,3))]);
figure;
plot(errorDegrees(10,:))
title(['Stance Ankle Roll kI = ', num2str(kI(10,1)),',',num2str(kI(10,2)),',',num2str(kI(10,3))]);
figure;
plot(sum(abs(errorDegrees(6:10,:))))
title('Right Leg');
figure;
plot(sum(abs(errorDegrees(1:6,:))))
title('Left Leg');