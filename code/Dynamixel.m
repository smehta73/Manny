%% Pre-conditions
% Cycle power on OpenCM9.04 controller

%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );

%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;
dxl_io.load_library();
dxl_io.connect(5, 1);

motor_ids = 1:22;

shoulder_pitch_left = 2;
shoulder_pitch_right = 1;
shoulder_yaw_left = 22;
shoulder_yaw_right = 21;
shoulder_roll_left = 4;
shoulder_roll_right = 3;

elbow_pitch_left = 6;
elbow_pitch_right = 5;

hip_roll_left = 10;
hip_roll_right = 9;
hip_yaw_left = 20;
hip_yaw_right = 19;
hip_pitch_left = 12; 
hip_pitch_right = 11;

knee_pitch_left = 14;
knee_pitch_right = 13;

ankle_roll_left = 18;
ankle_roll_right = 17;
ankle_pitch_left = 16;
ankle_pitch_right = 15;

% % %% Test Case - Trajectory: Raise left leg
%  target_pos_hip = 0;   % 60 degrees
%  target_speed =  pi/8;  % 2 sec. travel time
% % 
%  motor_ids_leftleg = [hip_roll_left hip_yaw_left hip_pitch_left knee_pitch_left ankle_roll_left ankle_pitch_left];
% target_pos_leftleg = zeros(1, length(motor_ids_leftleg));
% target_pos_leftleg(1) = target_pos_hip;
% % 
%  target_speed_leftleg = target_speed*ones(1, length(motor_ids_leftleg));
% % 
% % input('Lay Manny flat on his back. Press <Enter> to kick left leg up.\n');
dxl_io.set_motor_pos_speed(10, 0, 0, pi/8 );
% 
pause(2.5);
pos_vel_final = dxl_io.read_present_pos_vel( motor_ids_leftleg, 1, 'pos')

