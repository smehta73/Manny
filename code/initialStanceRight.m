MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );
close all
clear all
dxl_io = Dynamixel_IO;
dxl_io.load_library();
dxl_io.connect(0, 1); 
load('initialStanceAlphas.mat');
load('IPPart1Alphas.mat');
load('IPPart2Alphas.mat');
load('IPPart3Alphas.mat');
motor_ids = [10,12,14,16,18,9,11,13,15,17];
a = [ initialStanceAlphas IPPart1Alphas IPPart2Alphas IPPart3Alphas];
load('Motor_Biases.mat');
Motor_Biases = -1* motor_bias;
motorBias =[Motor_Biases(2:2:10); Motor_Biases(1:2:9)];
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
start = 200;
initialPositions = [-a(1,start);a(2,start); a(3,start) ; a(4,start);a(5,start); -a(6,start);-a(7,start); a(8,start); -a(9,start);a(10,start)];
initialPositions = -circshift(initialPositions,5);
% initialPositions = [0.0010;0.0508;-0.0235;0.0270;0.0010;-0.0012;0.0720;0.8976;0.5364;-0.0012];
% initialPositions = -circshift(initialPositions,5);
dxl_io.set_motor_pos_speed(motor_ids, motorBias,initialPositions, ones(length(motor_ids),1)*pi/6);
pause(10)
currentMotorPosition = dxl_io.read_present_pos_vel(motor_ids', 1, 'pos');
