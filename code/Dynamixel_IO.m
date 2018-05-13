classdef Dynamixel_IO
    % High-level communication/command interface (serial) for Dynamixel motors
    % http://support.robotis.com/en/software/dynamixel_sdk/api_reference/dxl_initialize.htm
    
    properties  (Access = public)
        port = 5;
        baud = 1;
    end
    properties (Access = private)
        os = -1;                 % 0 = linux; 1 = windows; 2 = mac os
        library;                % DXL library name
        load_lib;               % Binary shared library filename
        
        % Dynamixel Constants
        % Dynamixel register addresses (low byte)
        P_MODEL_NUMBER      = 0;    % word
        P_FIRMWARE          = 2;    % byte
        P_ID                = 3;    % byte
        P_BAUD_RATE         = 4;    % byte
        P_RETURN_DELAY      = 5;    % byte
        P_CW_ANGLE_LIM      = 6;    % word
        P_CCW_ANGLE_LIM     = 8;    % word
        P_TEMP_LIM          = 11;   % byte
        P_VOLT_LIM_LOW      = 12;   % byte
        P_VOLT_LIM_HIGH     = 13;   % byte
        P_MAX_TORQUE        = 14;   % word
        P_STATUS_RET_LVL    = 16;   % byte
        P_ALARM_LED         = 17;   % byte
        P_ALARM_SHUTDOWN    = 18;   % byte
        P_TORQUE_ENABLE     = 24;   % byte
        P_LED_ON            = 25;   % byte
        P_CW_COMP_MARG      = 26;   % byte
        P_CCW_COMP_MARG     = 27;   % byte
        P_CW_COMP_SLOPE     = 28;   % byte
        P_CCW_COMP_SLOPE    = 29;   % byte
        P_GOAL_POSITION     = 30;   % word
        P_MOVING_SPEED      = 32;   % word
        P_TORQUE_LIM        = 34;   % word
        P_PRESENT_POSITION  = 36;   % word
        P_PRESENT_SPEED     = 38;   % word
        P_PRESENT_LOAD      = 40;   % word
        P_PRESENT_VOLTAGE   = 42;   % byte
        P_PRESENT_TEMP      = 43;   % byte
        P_REGISTERED        = 44;   % byte
        P_MOVING            = 46;   % byte
        P_LOCK_EEPROM       = 47;   % byte
        P_PUNCH             = 48;   % word
        
        % Dynamixel commands
        INST_PING		= 1;
        INST_READ       = 2;
        INST_WRITE      = 3;
        INST_REG_WRITE  = 4;
        INST_ACTION     = 5;
        INST_RESET      = 6;
        INST_SYNC_WRITE	= 131;
        
        % Dynamixel error codes
        ERRBIT_VOLTAGE      = 1;
        ERRBIT_ANGLE        = 2;
        ERRBIT_OVERHEAT     = 4;
        ERRBIT_RANGE        = 8;
        ERRBIT_CHECKSUM     = 16;
        ERRBIT_OVERLOAD     = 32;
        ERRBIT_INSTRUCTION  = 64;
        
        DXL_BROADCAST_ID    = 254;
%         MAXNUM_TXPARAM = 150;
%         MAXNUM_RXPARAM = 60;
        
        % Dynamixel communication status on last tx/rx (return from 'dxl_get_result()')
        COMM_TXSUCCESS  = 0;
        COMM_RXSUCCESS  = 1;
        COMM_TXFAIL     = 2;
        COMM_RXFAIL     = 3;
        COMM_TXERROR    = 4;
        COMM_RXWAITING  = 5;
        COMM_RXTIMEOUT  = 6;
        COMM_RXCORRUPT  = 7;
    end
    
    methods
        %% Initialization & connection member functions
        % Dynamixel_IO Constructor
        function obj = Dynamixel_IO( )
            if ( ismac == 1 )   % Mac OS
                obj.os = 2;
                obj.library = 'libdxl';
                obj.load_lib = 'libdxl.so';
                fprintf(1, 'Dynamixel_IO() - Instatiating (Mac OS detected)!\n');
            elseif ( isunix == 1 )  % Unix derivative
                obj.os = 0;
                obj.library = 'libdxl';
                obj.load_lib = 'libdxl.so';
                fprintf(1, 'Dynamixel_IO() - Instatiating (Unix OS detected)!\n');
            elseif ( ispc == 1 )    % Windows
                obj.os = 1;
                obj.library = 'dynamixel';
                obj.load_lib = obj.library;
                fprintf(1, 'Dynamixel_IO() - Instatiating (Windows OS detected)!\n');
            else
                error('ERROR: Dynamixel_IO() - Unsupported operating system detected!\n');
            end
        end
        
        % Add dynamixel library paths and load dynamixel library
        function load_library( obj, path_list )
            if ( exist('path_list', 'var') && iscellstr(path_list) )
                for path_num = 1:length(path_list)
                    fprintf(1, 'adding %s ...\n', path_list{path_num});
                    addpath( path_list{path_num} );
                end
            else
                dxl_path = which('Dynamixel_IO');
                if ( obj.os == 0 )   % Unix
                    [pathstr, name, ext] = fileparts(dxl_path);
                    addpath([pathstr '/../../dynamixel_sdk/DXL_SDK_LINUX_v1_01/include']);
                    addpath([pathstr '/../../dynamixel_sdk/DXL_SDK_LINUX_v1_01/lib']);
                elseif ( obj.os == 1 )   % Windows
                    base_dir_end_ind = strfind(dxl_path, 'matlab')-1;
                    addpath([dxl_path(1:base_dir_end_ind) 'dynamixel_sdk\dxl_sdk_win32_v1_02\import']);
                    addpath([dxl_path(1:base_dir_end_ind) 'dynamixel_sdk\dxl_sdk_win32_v1_02\bin']);
                elseif ( obj.os == 2 )   % Mac OS
                    [pathstr, name, ext] = fileparts(dxl_path);
                    addpath([pathstr '/../../dynamixel_sdk/DXL_SDK_MAC_v1_01/include']);
                    addpath([pathstr '/../../dynamixel_sdk/DXL_SDK_MAC_v1_01/lib']);
                end
            end
            
            loadlibrary(obj.load_lib,'dynamixel.h');
            libfunctions(obj.library, '-full');
        end
        
        % Open serial connection via FTDI drivers
        %   Windows:= 'COM##' 
        %   LINUX = '/dev/ttyUSB##'
        %
        %   Set port = ##
        %   Set baud according to below table:
        %       Baud Value:   Goal BPS:
        %       1             1000000.0
        %       3              500000.0
        %       4              400000.0
        %       7              250000.0
        %       9              200000.0
        %       16             115200.0
        %       34              57600.0
        %       103             19200.0
        %       207              9600.0
        function result = connect( obj, port, baud )
            obj.port = port;
            obj.baud = baud;
            
            result = 1;
            
            % Open dynamixel serial device/connection
            res = calllib(obj.library, 'dxl_initialize', obj.port, obj.baud);
            if res == 1
                fprintf(1, 'Dynamixel_IO.connect() - SUCCEEDED opening serial port: %d @ baudnum: %d! \n', obj.port, obj.baud);
            else
                fprintf(1, 'Dynamixel_IO.connect() - ERROR opening serial port: %d @ baudnum: %d! \n', obj.port, obj.baud);
                result = 0;
                return;
            end
        end
        
        % Disconnect/release serial port
        function result = disconnect( obj )
            result = 1;
            
            % Close dynamixel serial device/connection
            calllib(obj.library,'dxl_terminate');
        end
        
        % Unload dynamixel libraries
        function result = unload_library( obj )
            result = 1;
            
            % Close dynamixel serial device/connection
            unloadlibrary(obj.library);
        end
        
       %% Dynamixel high-level motor interface
        % Summary:
        %   Commands dynamixel motors of snake robot to execute position/velocity
        %   trajectory vs. time
        %
        %   Input:
        %       theta: 9-by-length(time) matrix ( 9 motors, length(time) pos.
        %               values) [range: -150*pi/180 to +150*pi/180, units: rad]
        %       vel: 9-by-length(time) matrix ( 9 motors, length(time) vel values)
        %               [range: 0 to 85*2*pi/60 (ie. 85 rpm), units: rad/s]
        %       time: array of timestamps corresponding to each associated pos &
        %               vel [sec]
        %
        % DEPRECRATED (05/14/2015)
        function [ theta_actual ] = execute_snake_trajectory( obj, read_pos, motor_ids, theta, vel, time, joint_compl_margin, joint_bias )
            assert( (size(theta, 1) == length(motor_ids) ), 'Size mismatch: theta and motor_ids!');
            assert( (size(vel, 1) == length(motor_ids) ), 'Size mismatch: vel and motor_ids!');
            %             assert( (size(joint_compl_margin, 1) == (length(motor_ids)+1)/2 ), 'Size mismatch: joint_compl_margin and motor_ids!');
            assert( (read_pos == 1 || read_pos == 0 ), 'Invalid arg. value: read_pos must be 1 or 0!');
            
            NUM_MOTORS = length(motor_ids);
            
            motor_pos_lim = [-150 150]*pi/180;     % (rad) [0-1023] counts
            rad_to_motor_pos = 1023/(motor_pos_lim(2)-motor_pos_lim(1));   % motor pos counts per rad
            
            motor_vel_lim = [0 85*2*pi/60];     % (rad/s) [0-765] counts
            % (0.0116 rad/s / count)
            rad_s_to_motor_vel = 86;            % motor vel counts per rad/s (ie. 1/0.0116)
            
            % input parameter 'pre-processing' (convert rad & rad/s to int32 motor command values)
            theta = int32( (theta+150*pi/180)*rad_to_motor_pos );
            vel = int32( abs(vel)*rad_s_to_motor_vel );
            
            delta_time = [ 0 time(2:end)-time(1:end-1) ];
            NUM_TIME_STEPS = length(delta_time);
            
            theta_actual = zeros(NUM_MOTORS, NUM_TIME_STEPS);
            % Execute trajectory
            for time_index = 1:NUM_TIME_STEPS
                % Command joint positions and velocities (sync write)
                if exist( 'joint_bias', 'var' )
                    obj.set_motor_pos_speed( motor_ids, joint_bias, theta(:, time_index), vel(:, time_index) );
                else
                    obj.set_motor_pos_speed( motor_ids, zeros(length(motor_ids, 1)), theta(:, time_index), vel(:, time_index) );
                end
                
                % Set vertical motor compliance
                if exist( 'joint_compl_margin', 'var' )
                    vert_motor_id = motor_ids(1:2:end);
                    
                    obj.set_compliance_margin( vert_motor_id, joint_compl_margin(:, time_index) );
                end
                
                % time delay between joint commands
%                 pause(delta_time(time_index)*1.0);
                                pause(delta_time(time_index)*0.95);  % lateral undulation / compl slope = 8
                %                 pause(delta_time(time_index)*0.9); % sidewinding / compl slope = 8
                %                   pause(delta_time(time_index)*0.75); % concertina / compl slope = 8
                
                % Read joint positions
                if ( read_pos == 1 )
                    [ result ] = obj.dxl_read_register(motor_ids, [36], [2], 0);
                    theta_actual(:, time_index) = result/rad_to_motor_pos-150*pi/180;
                end
            end
            
            % Clean-up
            fprintf(1, 'Trajectory execution COMPLETE!\n');
            
            % safety margin -> allow completion of last motor command(s)
            pause(0.5);
        end
        
        % Command trajectory execution
        % 
        % Input(s):
        %   read_pos - 1, 0 (do, don't ready current position at each time step) 
        %   motor_ids - n-vector of motor IDs to command
        %   joint_bias - n-vector of joint biases/offsets (0-255) for each motor in motor_ids
        %   theta - n-by-m matrix of goal positions (rad)
        %   speed - n-by-m matrix of goal velocities (+rad/s)
        %   time - n-by-m matrix of timestamps dictating time step sizes (sec)
        %   compl_margin - n-by-m matrix of motor compliance margin (dead zone) values (0-255)
        % 
        % Output(s):
        %   theta_actual - nxm matrix of measured motor positions at each
        %                  time step of execution
        function [ theta_actual ] = execute_trajectory( obj, read_pos, motor_ids, joint_bias, theta, speed, time, compl_margin )
            assert( (size(theta, 1) == length(motor_ids) ), 'Size mismatch: theta and motor_ids!');
            assert( (size(speed, 1) == length(motor_ids) ), 'Size mismatch: speed and motor_ids!');
            assert( (size(compl_margin, 1) == length(motor_ids) ), 'Size mismatch: compl_margin and motor_ids!');
            assert( (read_pos == 1 || read_pos == 0 ), 'Invalid arg. value: read_pos must be 1 or 0!');
            
            NUM_MOTORS = length(motor_ids);
            
            delta_time = [ 0 time(2:end)-time(1:end-1) ];
            NUM_TIME_STEPS = length(delta_time);
            
            theta_actual = zeros(NUM_MOTORS, NUM_TIME_STEPS);
            % Execute trajectory
            for time_index = 1:NUM_TIME_STEPS
                % Command joint positions and velocities (sync write)
                obj.set_motor_pos_speed( motor_ids, joint_bias, theta(:, time_index), speed(:, time_index) );
                
                % Set vertical motor compliance
                if exist( 'compl_margin', 'var' )
                    obj.set_compliance_margin( motor_ids, compl_margin(:, time_index) );
                end
                
                % time delay between joint commands
                pause(delta_time(time_index)*1);
                
                % Read joint positions
                if ( read_pos == 1 )
                    theta_actual(:, time_index) = obj.read_present_pos_vel(motor_ids, 0, 'pos');
                end
            end
            
            % Clean-up
            fprintf(1, 'Trajectory execution COMPLETE!\n');
        end
        
        % Guides user through calibration procedure to determine motor
        % offsets/biases at zero'd horn position 
        % 
        % Input(s):
        %     motor_ids - n-vector of motor IDs to calibrate
        %     print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   motor_bias - n-vector of counter-clockwise ticks in excess of
        %                zero position (0-255)
        function motor_bias = calibrate_motor( obj, motor_ids, print_results )
            input(['Zero the following motors IDs. Press <Enter> when complete: ' int2str(motor_ids)]);
            motor_bias = obj.dxl_read_register(motor_ids, 36, 2, 0) - 512;
            
            if ( print_results )
                for index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d, Position: %.2f (deg.)\n', motor_ids(index), motor_bias(index)*300/1023);
                end
            end
        end
        
        % Simultaneously (SYNC_WRITE) command goal positions and velocities
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to command
        %   motor_bias - n-vector of joint biases/offsets (0-255) for each motor in motor_ids
        %   pos - n-vector of goal positions (rad)
        %   speed - n-vector of goal velocities (+rad/s)
        function comm_status = set_motor_pos_speed( obj, motor_ids, motor_bias, pos, speed )
            assert( (size(pos, 1) == length(motor_ids) ), 'Size mismatch: pos and motor_ids!');
            
            num_bytes = 2;
            if ( exist('speed', 'var') )
                assert( (size(speed, 1) == length(motor_ids) ), 'Size mismatch: speed and motor_ids!');
                num_bytes = 4;
            end
            
            NUM_MOTORS = length(motor_ids);
            data = zeros(NUM_MOTORS, num_bytes);
            
            pos = int32(obj.rad_to_pos_cnt(pos));
            speed = int32(obj.rad_s_to_speed_cnt(abs(speed)));
            
            % Apply correction for motor bias (assume 'motor_bias'
            % generated by Dynamixel_IO::calibrate_motor( ... ) function
            pos = pos - int32(motor_bias);
            
            for index = 1:NUM_MOTORS
                data(index, 1) = calllib(obj.library,'dxl_get_lowbyte', pos(index));
                data(index, 2) = calllib(obj.library,'dxl_get_highbyte', pos(index));
                if ( num_bytes == 4 )
                    data(index, 3) = calllib(obj.library,'dxl_get_lowbyte', speed(index));
                    data(index, 4) = calllib(obj.library,'dxl_get_highbyte', speed(index));
                end
            end
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_GOAL_POSITION, data );
        end
        
        % Simultaneously set (SYNC_WRITE) compliance margin
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   compl_margin - n-vector of motor compliance margin values (0-255)
        %   direction - 'cw', 'ccw'
        function comm_status = set_compliance_margin( obj, motor_ids, compl_margin, direction )
            assert( (size(compl_margin, 1) == length(motor_ids) ), 'Size mismatch: compl_margin and motor_ids!');
            
            if ( exist('direction', 'var') )
                if ( strcmp(direction, 'cw') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_CW_COMP_MARG, int32(compl_margin) );
                elseif ( strcmp(direction, 'ccw') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_CCW_COMP_MARG, int32(compl_margin) );
                else
                    error('Dynamixel_IO::set_compliance_marg() - Invalid input value for direction!');
                end
            else
                comm_status = obj.dxl_sync_write( motor_ids, obj.P_CW_COMP_MARG, int32(compl_margin) );
            end
        end
        
        % Simultaneously set (SYNC_WRITE) compliance slope
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   compl_slope - n-vector of motor compliance slope values (2, 4, 8, ..., 128)
        %   direction - 'cw', 'ccw'
        function comm_status = set_compliance_slope( obj, motor_ids, compl_slope, direction )
            assert( (size(compl_slope, 1) == length(motor_ids) ), 'Size mismatch: compl_slope and motor_ids!');
            
            if ( exist('direction', 'var') )
                if ( strcmp(direction, 'cw') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_CW_COMP_SLOPE, compl_slope );
                elseif ( strcmp(direction, 'ccw') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_CCW_COMP_SLOPE, compl_slope );
                else
                    error('Dynamixel_IO::set_compliance_slope() - Invaid input value for direction!');
                end
            else
                comm_status = obj.dxl_sync_write( motor_ids, obj.P_CW_COMP_SLOPE, compl_slope );
            end
        end
        
        % Simultaneously set (SYNC_WRITE) punch
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   punch - n-vector of motor punch values (32-1023)
        function comm_status = set_punch( obj, motor_ids, punch )
            assert( (size(punch, 1) == length(motor_ids) ), 'Size mismatch: punch and motor_ids!');
            
            data = zeros( length(motor_ids), 2 );
            for index = 1:length(motor_ids)
                data(index, 1) = calllib(obj.library,'dxl_get_lowbyte', int32(punch(index)));
                data(index, 2) = calllib(obj.library,'dxl_get_highbyte', int32(punch(index)));
            end
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_PUNCH, data );
        end
        
        % Simultaneously set (SYNC_WRITE) torque enable
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   torque_enable - n-vector of torque enable values (1, 0)
        function comm_status = set_torque_enable( obj, motor_ids, torque_enable )
            assert( (size(torque_enable, 1) == length(motor_ids) ), 'Size mismatch: torque_enable and motor_ids!');
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_TORQUE_ENABLE, torque_enable );
        end
        
        % Simultaneously set (SYNC_WRITE) LED
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   led_state - n-vector of led on/off values (1, 0)
        function comm_status = set_led( obj, motor_ids, led_state )
            assert( (size(led_state, 1) == length(motor_ids) ), 'Size mismatch: led_state and motor_ids!');
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_LED_ON, led_state );
        end
        
        % Simultaneously set (SYNC_WRITE) Torque Limit
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   torque_limit - n-vector of torque limit values (0-100%)
        function comm_status = set_torque_limit( obj, motor_ids, torque_limit )
            assert( (size(torque_limit, 1) == length(motor_ids) ), 'Size mismatch: torque_limit and motor_ids!');
            
            torque_limit = torque_limit/100*1023;
            
            data = zeros( length(motor_ids), 2 );
            for index = 1:length(motor_ids)
                data(index, 1) = calllib(obj.library,'dxl_get_lowbyte', int32(torque_limit(index)));
                data(index, 2) = calllib(obj.library,'dxl_get_highbyte', int32(torque_limit(index)));
            end
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_TORQUE_LIM, data );
        end
        
        % Simultaneously set (SYNC_WRITE) Max. Torque
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   max_torque - n-vector of max. torque values (0-100%)
        function comm_status = set_max_torque( obj, motor_ids, max_torque )
            assert( (size(max_torque, 1) == length(motor_ids) ), 'Size mismatch: max_torque and motor_ids!');
            
            max_torque = max_torque/100*1023;
            
            data = zeros( length(motor_ids), 2 );
            for index = 1:length(motor_ids)
                data(index, 1) = calllib(obj.library,'dxl_get_lowbyte', int32(max_torque(index)));
                data(index, 2) = calllib(obj.library,'dxl_get_highbyte', int32(max_torque(index)));
            end
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_MAX_TORQUE, data );
        end
        
        % Simultaneously set (SYNC_WRITE) voltage limit
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   voltage_limit - n-vector of voltage limit values (5-25)
        %   limit_type - 'low', 'high'
        function comm_status = set_voltage_limit( obj, motor_ids, voltage_limit, limit_type )
            assert( (size(voltage_limit, 1) == length(motor_ids) ), 'Size mismatch: voltage_limit and motor_ids!');
            
            voltage_limit = int32(voltage_limit*10);
            
            if ( exist('limit_type', 'var') )
                if ( strcmp(limit_type, 'low') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_VOLT_LIM_LOW, voltage_limit );
                elseif ( strcmp(limit_type, 'high') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_VOLT_LIM_HIGH, voltage_limit );
                else
                    error('Dynamixel_IO::set_voltage_limit() - Invaid input value for limit_type!');
                end
            else
                comm_status = obj.dxl_sync_write( motor_ids, obj.P_VOLT_LIM_LOW, voltage_limit );
            end
        end
        
        % Simultaneously set (SYNC_WRITE) highest temperature limit
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   temp_limit - n-vector of voltage_limit values (default 80 deg. C)
        function comm_status = set_temp_limit( obj, motor_ids, temp_limit )
            assert( (size(temp_limit, 1) == length(motor_ids) ), 'Size mismatch: temp_limit and motor_ids!');
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_TEMP_LIM, int32(temp_limit) );
        end
        
        % Simultaneously set (SYNC_WRITE) joint angle lmits (cw and/or ccw)
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   angle_limit - n-vector of joint angle limit values (radian equivalent of -150 to 149.7070 degrees)
        %   limit_type - 'cw', 'ccw'
        function comm_status = set_angle_limit( obj, motor_ids, angle_limit, direction )
            assert( (size(angle_limit, 1) == length(motor_ids) ), 'Size mismatch: angle_limit and motor_ids!');
            
            angle_limit = obj.rad_to_pos_cnt(angle_limit);
            
            data = zeros( length(motor_ids), 2*size(angle_limit, 2) );
            for index = 1:length(motor_ids)
                data(index, 1) = calllib(obj.library,'dxl_get_lowbyte', int32(angle_limit(index, 1)));
                data(index, 2) = calllib(obj.library,'dxl_get_highbyte', int32(angle_limit(index, 1)));
                if ( size(angle_limit, 2) > 1 )
                    data(index, 3) = calllib(obj.library,'dxl_get_lowbyte', int32(angle_limit(index, 2)));
                    data(index, 4) = calllib(obj.library,'dxl_get_highbyte', int32(angle_limit(index, 2)));
                end
            end
            
            if ( exist('direction', 'var') )
                if ( strcmp(direction, 'cw') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_CW_ANGLE_LIM, data );
                elseif ( strcmp(direction, 'ccw') )
                    comm_status = obj.dxl_sync_write( motor_ids, obj.P_CCW_ANGLE_LIM, data );
                else
                    error('Dynamixel_IO::set_angle_limit() - Invaid input value for direction!');
                end
            else
                comm_status = obj.dxl_sync_write( motor_ids, obj.P_CW_ANGLE_LIM, data );
            end
        end
        
        % Simultaneously set (SYNC_WRITE) alarm LED
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   alarm_led_conf - n-vector of alarm LED enable/disable values (byte flag)
        function comm_status = set_alarm_led( obj, motor_ids, alarm_led_conf )
            assert( (size(alarm_led_conf, 1) == length(motor_ids) ), 'Size mismatch: alarm_led_conf and motor_ids!');
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_ALARM_LED, alarm_led_conf );
        end
        
        % Simultaneously set (SYNC_WRITE) alarm shutdown
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   alarm_shutdown_conf - n-vector of alarm shutdown enable/disable values (byte flag)
        function comm_status = set_alarm_shutdown( obj, motor_ids, alarm_shutdown_conf )
            assert( (size(alarm_shutdown_conf, 1) == length(motor_ids) ), 'Size mismatch: alarm_shutdown_conf and motor_ids!');
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_ALARM_SHUTDOWN, alarm_shutdown_conf );
        end
        
        % Simultaneously set (SYNC_WRITE) new motor IDs
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to configure
        %   alarm_shutdown_conf - n-vector of new motor ID values (0-252)
        function comm_status = set_id( obj, motor_ids, new_id )
            assert( (size(new_id, 1) == length(motor_ids) ), 'Size mismatch: new_id and motor_ids!');
            
            comm_status = obj.dxl_sync_write( motor_ids, obj.P_ID, new_id );
        end
        
        % Retrieve motor moving status
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor moving status values (1, 0)
        function result = is_moving( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, obj.P_MOVING, 1, 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d is moving: %d\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor temperature
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor temperature (deg. C)
        function result = read_present_temp( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, obj.P_PRESENT_TEMP, 1, 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d present temp.: %d deg. C\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor voltage
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor voltage (V)
        function result = read_present_volt( obj, motor_ids, print_results )
            result = double(obj.dxl_read_register( motor_ids, obj.P_PRESENT_VOLTAGE, 1, 0 ))/10;
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d present volt.: %.2f V\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor load
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor load (%)
        function result = read_present_load( obj, motor_ids, print_results )
            result = obj.load_cnt_to_percent(double(obj.dxl_read_register( motor_ids, obj.P_PRESENT_LOAD, 2, 0 )));
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d present load = %.2f perc.\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end

        % Retrieve motor position and/or speed
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        %   type - 'pos', 'speed' (or optionally not specified)
        % 
        % Output(s):
        %   result - 
        %       if type = 'pos' or 'speed': n-vector of motor pos or speed (rad or rad/s)
        %       if type not specified: n-by-2 matrix of motor pos (col. 1) and speed (col. 2)
        function result = read_present_pos_vel( obj, motor_ids, print_results, type )
            if ( exist('type', 'var') )
                if ( strcmp(type, 'pos' ) )
                    result = obj.pos_cnt_to_rad(double(obj.dxl_read_register( motor_ids, obj.P_PRESENT_POSITION, 2, 0 )));
                elseif ( strcmp(type, 'speed') )
                    result = obj.vel_cnt_to_rad_s(double(obj.dxl_read_register( motor_ids, obj.P_PRESENT_SPEED, 2, 0 )));
                end
            else
                result = obj.dxl_read_register( motor_ids, [ obj.P_PRESENT_POSITION, obj.P_PRESENT_SPEED ], [ 2, 2 ], 0 );
                result(:, 1) = obj.pos_cnt_to_rad(double(result(:, 1)));
                result(:, 2) = obj.vel_cnt_to_rad_s(double(result(:, 2)));
            end
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    if ( exist('type', 'var') )
                        fprintf(1, 'Motor ID: %d present %s: %.4f rad or rad/s\n', motor_ids(motor_index), type, result(motor_index, 1));
                    else
                        fprintf(1, 'Motor ID: %d present %s: %.4f rad\n', motor_ids(motor_index), 'pos', result(motor_index, 1));
                        fprintf(1, 'Motor ID: %d present %s: %.4f rad/s\n', motor_ids(motor_index), 'speed', result(motor_index, 2));
                    end
                end
            end
        end
           
        % Retrieve motor information
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-by-4 matrix of motor information (model #, firmware
        %            ver., motor ID, motor baud rate)
        function result = read_motor_info( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, [ obj.P_MODEL_NUMBER, obj.P_FIRMWARE, obj.P_ID, obj.P_BAUD_RATE ], ...
                [2, 1, 1, 1], 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d - \n\t Model Number: %d \n\t Firmware Version: %d \n\t Baud Rate: %d \n\n', ...
                        result(motor_index, 3), result(motor_index, 1), result(motor_index, 2), result(motor_index, 4));
                end
            end
        end
        
        % Retrieve motor alarm LED enabled/disabled status
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor alarm LED enabled/disabled status (byte flag)
        function result = read_alarm_led( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, obj.P_ALARM_LED, 1, 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d alarm LED config.: %d\n', motor_ids(motor_index), result(motor_index));
                    msg = [ 'Input Voltage Error: ' int2str(bitand(result(motor_index), 1)>0) '\nAngle Limit Error: ' int2str(bitand(result(motor_index), 2)>0) ...
                        '\nOverHeating Error: ' int2str(bitand(result(motor_index), 4)>0) '\nRange Error: ' int2str(bitand(result(motor_index), 8)>0) ...
                        '\nCheckSum Error: ' int2str(bitand(result(motor_index), 16)>0) '\nOverload Error: ' int2str(bitand(result(motor_index), 32)>0) ...
                        '\nInstruction Error: ' int2str(bitand(result(motor_index), 64)>0) '\n\n' ];
                    fprintf( 1, msg );
                end
            end
        end
        
        % Retrieve motor alarm shutdown enabled/disabled status
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor alarm shutdown enabled/disabled status (byte flag)
        function result = read_alarm_shutdown( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, obj.P_ALARM_SHUTDOWN, 1, 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d alarm shutdown config.: %d\n', motor_ids(motor_index), result(motor_index));
                    msg = [ 'Input Voltage Error: ' int2str(bitand(result(motor_index), 1)>0) '\nAngle Limit Error: ' int2str(bitand(result(motor_index), 2)>0) ...
                        '\nOverHeating Error: ' int2str(bitand(result(motor_index), 4)>0) '\nRange Error: ' int2str(bitand(result(motor_index), 8)>0) ...
                        '\nCheckSum Error: ' int2str(bitand(result(motor_index), 16)>0) '\nOverload Error: ' int2str(bitand(result(motor_index), 32)>0) ...
                        '\nInstruction Error: ' int2str(bitand(result(motor_index), 64)>0) '\n\n' ];
                    fprintf( 1, msg );
                end
            end
        end
        
        % Retrieve motor punch value
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor punch values (%)
        function result = read_punch( obj, motor_ids, print_results )
            result = double(obj.dxl_read_register( motor_ids, obj.P_PUNCH, 2, 0 ))/10;
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d punch: %.2f perc.\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor torque limit
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor torque limits (%)
        function result = read_torque_limit( obj, motor_ids, print_results )
            result = double(obj.dxl_read_register( motor_ids, obj.P_TORQUE_LIM, 2, 0 ))/1023*100;
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d torque limit: %.2f perc.\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor max. torque
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor max. torque values (%)
        function result = read_max_torque( obj, motor_ids, print_results )
            result = double(obj.dxl_read_register( motor_ids, obj.P_MAX_TORQUE, 2, 0 ))/1023*100;
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d max. torque: %.2f perc.\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor angle limit(s)
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        %   direction - 'cw', 'ccw' (or optionally not specified)
        % 
        % Output(s):
        %   result - 
        %       if direction = 'cw' or 'ccw': n-vector of motor cw or ccw angle limit (rad)
        %       if direction not specified: n-by-2 matrix of motor cw (col. 1) and ccw (col. 2) angle limits
        function result = read_angle_limit( obj, motor_ids, print_results, direction )
            if ( exist('direction', 'var') )
                if ( strcmp(direction, 'cw') )
                    result = obj.pos_cnt_to_rad(double(obj.dxl_read_register( motor_ids, obj.P_CW_ANGLE_LIM, 2, 0 )));
                elseif ( strcmp(direction, 'ccw') )
                    result = obj.pos_cnt_to_rad(double(obj.dxl_read_register( motor_ids, obj.P_CCW_ANGLE_LIM, 2, 0 )));
                end
            else
                result = obj.pos_cnt_to_rad(double(obj.dxl_read_register( motor_ids, [ obj.P_CW_ANGLE_LIM, obj.P_CCW_ANGLE_LIM ], [ 2, 2 ], 0 )));
            end
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    if ( exist('direction', 'var') )
                        fprintf(1, 'Motor ID: %d angle limit (%s): %.2f rad\n', motor_ids(motor_index), direction, result(motor_index, 1));
                    else
                        fprintf(1, 'Motor ID: %d angle limit (%s): %.2f rad\n', motor_ids(motor_index), 'cw', result(motor_index, 1));
                        fprintf(1, 'Motor ID: %d angle limit (%s): %.2f rad\n', motor_ids(motor_index), 'ccw', result(motor_index, 2));
                    end
                end
            end
        end
        
        % Retrieve motor compliance margin(s)
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        %   direction - 'cw', 'ccw' (or optionally not specified)
        % 
        % Output(s):
        %   result - 
        %       if direction = 'cw' or 'ccw': n-vector of motor cw or ccw compliance margin (rad)
        %       if direction not specified: n-by-2 matrix of motor cw (col. 1) and ccw (col. 2) compliance margins (rad)
        function result = read_compliance_margin( obj, motor_ids, print_results, direction )
            if ( exist('direction', 'var') )
                if ( strcmp(direction, 'cw') )
                    result = double(obj.dxl_read_register( motor_ids, obj.P_CW_COMP_MARG, 1, 0 ))*300/1024*pi/180;
                elseif ( strcmp(direction, 'ccw') )
                    result = double(obj.dxl_read_register( motor_ids, obj.P_CCW_COMP_MARG, 1, 0 ))*300/1024*pi/180;
                end
            else
                result = double(obj.dxl_read_register( motor_ids, [ obj.P_CW_COMP_MARG, obj.P_CCW_COMP_MARG ], [ 1, 1 ], 0 ))*300/1024*pi/180;
            end
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    if ( exist('direction', 'var') )
                        fprintf(1, 'Motor ID: %d compliance margin (%s): %.2f rad\n', motor_ids(motor_index), direction, result(motor_index, 1));
                    else
                        fprintf(1, 'Motor ID: %d compliance margin (%s): %.2f rad\n', motor_ids(motor_index), 'cw', result(motor_index, 1));
                        fprintf(1, 'Motor ID: %d compliance margin (%s): %.2f rad\n', motor_ids(motor_index), 'ccw', result(motor_index, 2));
                    end
                end
            end
        end
        
        % Retrieve motor compliance slope(s)
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        %   direction - 'cw', 'ccw' (or optionally not specified)
        % 
        % Output(s):
        %   result - 
        %       if direction = 'cw' or 'ccw': n-vector of motor cw or ccw compliance slope (rad)
        %       if direction not specified: n-by-2 matrix of motor cw (col. 1) and ccw (col. 2) compliance slopes (rad)
        function result = read_compliance_slope( obj, motor_ids, print_results, direction )
            if ( exist('direction', 'var') )
                if ( strcmp(direction, 'cw') )
                    result = obj.dxl_read_register( motor_ids, obj.P_CW_COMP_SLOPE, 1, 0 );
                elseif ( strcmp(direction, 'ccw') )
                    result = obj.dxl_read_register( motor_ids, obj.P_CCW_COMP_SLOPE, 1, 0 );
                end
            else
                result = obj.dxl_read_register( motor_ids, [ obj.P_CW_COMP_SLOPE, obj.P_CCW_COMP_SLOPE ], [ 1, 1 ], 0 );
            end       
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    if ( exist('direction', 'var') )
                        fprintf(1, 'Motor ID: %d compliance slope (%s): %d\n', motor_ids(motor_index), direction, result(motor_index, 1));
                    else
                        fprintf(1, 'Motor ID: %d compliance slope (%s): %d\n', motor_ids(motor_index), 'cw', result(motor_index, 1));
                        fprintf(1, 'Motor ID: %d compliance slope (%s): %d\n', motor_ids(motor_index), 'ccw', result(motor_index, 2));
                    end
                end
            end
        end
        
        % Retrieve motor voltage limit(s)
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        %   limit_type - 'low', 'high' (or optionally not specified)
        % 
        % Output(s):
        %   result - 
        %       if limit_type = 'low' or 'high' n-vector of motor low or high voltage limit (V)
        %       if limit_type not specified: n-by-2 matrix of motor low (col. 1) and high (col. 2) voltage limits (rad)
        function result = read_voltage_limit( obj, motor_ids, print_results, limit_type )
            if ( exist('limit_type', 'var') )
                if ( strcmp(limit_type, 'low') )
                    result = double(obj.dxl_read_register( motor_ids, obj.P_VOLT_LIM_LOW, 1, 0 ))*0.1;
                elseif ( strcmp(limit_type, 'high') )
                    result = double(obj.dxl_read_register( motor_ids, obj.P_VOLT_LIM_HIGH, 1, 0 ))*0.1;
                end
            else
                result = double(obj.dxl_read_register( motor_ids, [ obj.P_VOLT_LIM_LOW, obj.P_VOLT_LIM_HIGH ], [ 1, 1 ], 0 ))*0.1;
            end
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    if ( exist('limit_type', 'var') )
                        fprintf(1, 'Motor ID: %d voltage limit (%s): %.2f V\n', motor_ids(motor_index), limit_type, result(motor_index, 1));
                    else
                        fprintf(1, 'Motor ID: %d voltage limit (%s): %.2f V\n', motor_ids(motor_index), 'low', result(motor_index, 1));
                        fprintf(1, 'Motor ID: %d voltage limit (%s): %.2f V\n', motor_ids(motor_index), 'high', result(motor_index, 2));
                    end
                end
            end
        end

        % Retrieve motor position and/or speed
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        %   type - 'pos', 'speed' (or optionally not specified)
        % 
        % Output(s):
        %   result - 
        %       if type = 'pos' or 'speed' n-vector of motor position or speed values (rad or rad/s)
        %       if type not specified: n-by-2 matrix of motor position (col. 1) and speed (col. 2) values (rad and rad/s)
        function result = read_goal_pos_speed( obj, motor_ids, print_results, type )
            if ( exist('type', 'var') )
                if ( strcmp(type, 'pos') )
                    result = obj.pos_cnt_to_rad(double(obj.dxl_read_register( motor_ids, obj.P_GOAL_POSITION, 2, 0 )));
                elseif ( strcmp(type, 'speed') )
                    result = obj.vel_cnt_to_rad_s(double(obj.dxl_read_register( motor_ids, obj.P_MOVING_SPEED, 2, 0 )));
                end
            else
                result = obj.dxl_read_register( motor_ids, [ obj.P_GOAL_POSITION, obj.P_MOVING_SPEED ], [ 2, 2 ], 0 );
                result(:, 1) = obj.pos_cnt_to_rad(double(result(:, 1)));
                result(:, 2) = obj.vel_cnt_to_rad_s(double(result(:, 2)));
            end
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    if ( exist('type', 'var') )
                        fprintf(1, 'Motor ID: %d goal %s: %.2f rad or rad/s\n', motor_ids(motor_index), type, result(motor_index, 1));
                    else
                        fprintf(1, 'Motor ID: %d goal %s: %.2f rad\n', motor_ids(motor_index), 'pos', result(motor_index, 1));
                        fprintf(1, 'Motor ID: %d goal %s: %.2f rad/s\n', motor_ids(motor_index), 'speed', result(motor_index, 2));
                    end
                end
            end
        end
        
        % Retrieve motor torque enable status
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor torque enabled values (1, 0)
        function result = read_torque_enable( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, obj.P_TORQUE_ENABLE, 1, 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d torque enable: %d\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor LED on/off status
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor LED on/off values (1, 0)
        function result = read_led( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, obj.P_LED_ON, 1, 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d LED on: %d\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        % Retrieve motor temperature limit
        % 
        % Input(s):
        %   motor_ids - n-vector of motor IDs to read from
        %   print_results - 1, 0 (do, don't print string-formatted results)
        % 
        % Output(s):
        %   result - n-vector of motor temperature limits (deg. C)
        function result = read_temp_limit( obj, motor_ids, print_results )
            result = obj.dxl_read_register( motor_ids, obj.P_TEMP_LIM, 1, 0 );
            
            if ( print_results )
                for motor_index = 1:length(motor_ids)
                    fprintf(1, 'Motor ID: %d highest temp. limit: %d\n', motor_ids(motor_index), result(motor_index));
                end
            end
        end
        
        %% Dynamixel low-level motor communications helper functions
        
        % For each motor in 'motor_ids', reads all locations in 'address',
        % retrieving number of bytes = value in appropriate row of 'num_bytes'
        %   Parameter(s):
        %       motor_id - array of motor id's against which all addresses will be read
        %       address - array of register addresses from which data will be read (for
        %                   each motor)
        %       num_bytes - array of values (1 or 2) indicating how many bytes to read
        %                   from each register address
        %
        %   Reads and prints present position (address 36, 2 bytes) and present
        %       speed (address 38, 2 bytes) from each of motor ID's 1 and 2.
        %   eg. read_dxl_register([1 2], [36, 38], [2 2], 1);
        %   Read motor compliance value(s)
        %   eg. read_dxl_register([0:11], [26:29 48], [1 1 1 1 1], 1);
        %   Read motor temperature(s) (internal temp. limit = 80 deg. Cel.)
        %   eg. read_dxl_register([0:11], 43, 1, 1);
        function [ result ] = dxl_read_register( obj, motor_ids, address, num_bytes, print_results )
            assert( (length(num_bytes) == length(address) ), 'Size mismatch: num_bytes and address!');
            
            result = zeros(length(motor_ids), length(address));
            % read register values
            for motor_index = 1:length(motor_ids)
                for addr_index = 1:length(address)
                    if ( num_bytes(addr_index) == 2 )
                        command = 'dxl_read_word';
                    else
                        command = 'dxl_read_byte';
                    end
                    reg_value = calllib(obj.library, command, motor_ids(motor_index), address(addr_index));
                    result(motor_index, addr_index) = reg_value;
                    
                    if ( print_results == 1 )
                        fprintf(1, 'Motor ID: %d, Register Addr.: %d, Bytes: %d, Value: %d\n', motor_ids(motor_index), ...
                            address(addr_index), num_bytes(addr_index), reg_value);
                    end
                end
            end
        end
        
        % For each motor in 'motor_ids', write corresponding value in
        % 'motor_data' to corresponding location in 'address'
        %   Parameter(s):
        %       motor_ids - array of motor id's to which data will be
        %                   written
        %       address - array of register addresses to which data will be written
        %                   (each entry is associated with corresponding
        %                   motor id entry in 'motor_ids')
        %       num_bytes - array of values (1 or 2) indicating how many
        %                   bytes to write to each register address
        %                   (each entry is associated with corresponding
        %                   motor id entry in 'motor_ids')
        function dxl_write_register( obj, motor_ids, address, num_bytes, motor_data )
            assert( (length(num_bytes) == length(address) && length(address) == length(motor_data)), ...
                'Size mismatch: num_bytes, address, motor_data!');
            
            % write register values
            for motor_index = 1:length(motor_ids)
                if ( num_bytes(motor_index) == 2 )
                    command = 'dxl_write_word';
                else
                    command = 'dxl_write_byte';
                end
                calllib(obj.library, command, motor_ids(motor_index), address(motor_index), motor_data(motor_index));
            end
        end
        
        % Simultaneously write to registers (SYNC_WRITE) of mult. motors
        %   Parameter(s):
        %       motor_ids - array of motor id's; each array position is associated
        %                       with the corresponding row of 'motor_data'
        %       address - starting register address at which data will be written (for
        %                   each motor)
        %       motor_data - array of byte values to write, beginning at 'address' of each 
        %                       motor's register table (motor_data is array of dimension
        %                       'length(motor_ids)'-by-'number data bytes')
        function comm_status = dxl_sync_write( obj, motor_ids, address, motor_data )
            assert( (size(motor_data, 1) == length(motor_ids) ), 'Size mismatch: data and motor_ids!');
            
            motor_data_size = size(motor_data, 2);
            NUM_MOTORS = length(motor_ids);
            
            % Construct sync write packet
            calllib(obj.library,'dxl_set_txpacket_id', obj.DXL_BROADCAST_ID);
            calllib(obj.library,'dxl_set_txpacket_instruction', obj.INST_SYNC_WRITE);
            calllib(obj.library,'dxl_set_txpacket_parameter',0, address);
            calllib(obj.library,'dxl_set_txpacket_parameter',1, motor_data_size);
            
            for i = 1:NUM_MOTORS
                calllib(obj.library,'dxl_set_txpacket_parameter', 2+(1+motor_data_size)*(i-1), motor_ids(i));
                
                for byte_num = 1:motor_data_size
                    calllib(obj.library,'dxl_set_txpacket_parameter', ...
                        2+(1+motor_data_size)*(i-1)+byte_num, motor_data(i, byte_num));
                end
            end
            
            calllib(obj.library,'dxl_set_txpacket_length',(1+motor_data_size)*NUM_MOTORS+4);
            
            calllib(obj.library,'dxl_txrx_packet');
            comm_status = int32(calllib(obj.library,'dxl_get_result'));
        end     % dxl_sync_write
        
        % Report ping attempt on motor ID
        %   Parameter(s):
        %       motor_id - single motor ID
        %       print_results - 1, 0 (do, don't print string-formatted results)
        function [comm_status ] = dxl_ping( obj, motor_id, print_results )            
            calllib(obj.library,'dxl_ping', motor_id);

            comm_status = int32(calllib(obj.library,'dxl_get_result'));
            if ( comm_status == obj.COMM_RXSUCCESS )
                if ( print_results )
                    fprintf(1, '\nSUCCESSFUL ping to motor id %d: ', motor_id);
                end
                obj.dxl_retrieve_error( print_results );
            else
                if ( print_results )
                    fprintf(1, '\nUNSUCCESSFUL ping to motor id %d: ', motor_id);
                end
            end
        end
        
        % Report error conditions from the last recieved motor status/response
        %   Parameter(s):
        %       print_results - 1, 0 (do, don't print string-formatted results)
        function error = dxl_retrieve_error( obj, print_results )
            error = int32(0);
            
            error = error + ( int32(calllib(obj.library,'dxl_get_rxpacket_error', obj.ERRBIT_VOLTAGE)) );
            error = error + bitshift( int32(calllib(obj.library,'dxl_get_rxpacket_error', obj.ERRBIT_ANGLE)), 1 );
            error = error + bitshift( int32(calllib(obj.library,'dxl_get_rxpacket_error', obj.ERRBIT_OVERHEAT)), 2 );
            error = error + bitshift( int32(calllib(obj.library,'dxl_get_rxpacket_error', obj.ERRBIT_RANGE)), 3 );
            error = error + bitshift( int32(calllib(obj.library,'dxl_get_rxpacket_error', obj.ERRBIT_CHECKSUM)), 4 );
            error = error + bitshift( int32(calllib(obj.library,'dxl_get_rxpacket_error', obj.ERRBIT_OVERLOAD)), 5 );
            error = error + bitshift( int32(calllib(obj.library,'dxl_get_rxpacket_error', obj.ERRBIT_INSTRUCTION)), 6 );
            
            if ( print_results )
                msg = [ '\nERRBIT_VOLTAGE: ' int2str(bitand(error, 1)) '\nERRBIT_ANGLE: ' int2str(bitand(error, 2)) ...
                    '\nERRBIT_OVERHEAT: ' int2str(bitand(error, 4)) '\nERRBIT_RANGE: ' int2str(bitand(error, 8)) ...
                    '\nERRBIT_CHECKSUM: ' int2str(bitand(error, 16)) '\nERRBIT_OVERLOAD: ' int2str(bitand(error, 32)) ...
                    '\nERRBIT_INSTRUCTION: ' int2str(bitand(error, 64)) '\n\n' ];
                fprintf( 1, msg );
            end
        end
        
       %% Conversion utilities

        % Convert load (to V)
            % positive = ccw, negative = cw
        function result = load_cnt_to_percent( obj, data )
            dir = bitshift(bitand(data, 1024), -10);   % 1 = CW, 0 = CCW
            
            result = bitand(data, 1023)*(0.1).*( (-1).^dir );
        end
        
        % Convert velocity (motor ticks/sec to rad/s)
        %   positive = ccw, negative = cw
        function result = vel_cnt_to_rad_s( obj, data )
            dir = bitshift(bitand(data, 1024), -10);   % 1 = CW, 0 = CCW
            
            result = bitand(data, 1023)*(0.111*2*pi/60).*( (-1).^dir );
        end
        
        % Convert position (motor ticks to rad)
        function result = pos_cnt_to_rad( obj, data )
            result = (data - 512)*300/1024*pi/180;
        end
        
        % Convert speed (rad/s to motor ticks/sec)
            % positive = ccw, negative = cw
        function result = rad_s_to_speed_cnt( obj, data )
            result = data/(0.111*2*pi/60);
        end
        
        % Convert position (rad to motor ticks)
        function result = rad_to_pos_cnt( obj, data )
            result = data*180/pi*1024/300 + 512;
        end
        
    end     % methods
end

