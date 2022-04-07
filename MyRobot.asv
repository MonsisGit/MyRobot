classdef MyRobot < handle
    % Class to control custom Robot
    % Example usage:
    % robot = MyRobot();
    % robot.set_speed([0.1,0.1,0.1,0.1],true);
    % robot.set_torque_limit([1,1,1,1]);
    % robot.draw_robot();
    % robot.move_j(20,-90,0,50);
    % robot.disable_motors();
    
    % If library fails to load, unplug the robot from the computer and
    % clear the workspace/ restart the GUI
    
    properties (Access = private)
        lib_name = 'dxl_x64_c';                     % Library name for Win10
        
        % Control table address
        ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address for enabling torque mode
        ADDR_MX_GOAL_POSITION       = 30;           % Control table address for reading goal position
        ADDR_MX_PRESENT_POSITION    = 36;           % Control table address for reading current position
        
        % Protocol version
        PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

        
        BAUDRATE                    = 1000000;      % Baudrate for Motors
        DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
        % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
        
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        
        port_num=0;                                 % Portnumber gets automatically handled by Porthandler
    end
    properties (Access=public)
        motor_ids = [0 1 2 3];                      % Motor IDs chronologically (see Dynamixel Wizard for more info)
        gripper_motor_id = 4                        % ID of gripper motor
        dh = [0   	-pi/2	0.0955 0;               % Denavit Hartenberg Parameters for Robot (a, alpha, d, theta)
            0.116	0       0       0;
            0.096	0	0	0;
            0.064  	0	0	0];
        
       
        forward_transform = zeros(4,4);             % Forward transformation Matrix        
        joint_angles = [0 0 0 0];                   % Internal joint angles in degree
        joint_pos = zeros(4,4);                     % Internal joint positions calculated with each move_j        
        draw_robot_flag = 0;                        % Flag for drawing robot configuration
        use_smooth_speed_flag = 0;                  % Flag for using smooth speed 
        gripper_open_flag = 1;                      % Flag for gripper status
        rbt = 0;                                    % RigidBodyTree        
        joint_limits = [-130 130; -180 0; -100 100; -100 100]; %Joint Limits in degree        
        ik = 0;                                     % Inverse Kinematics Object
        ik_weights = [0.25 0.25 0.25 1 1 1];        % Weights for inverse kinematics 
        joint_offsets = [171-5 150+90 150 150];     % Joint offsets to send to motor
        joint_angle_error = [0 0 0 0];              % Internal joint angle error between read out of joint angles and input joint angles
        init_status = 0;                            % Initialization succesfull flag
        movement_history = [];                      % List to record movement history
        motor_speed = 0;                            % List for motor speed, e.g. [0.2,0.1,0.1,1]
        motor_torque = 0;                           % List for motor torque, e.g. [1,1,0.5,0.5]
        pitch = 0;                                  % Pitch Angle for motor 3
   
    end
    methods
        function self = MyRobot()
            % Initialize robot, setting initial motor speeds to 10%, motor
            if ~libisloaded(self.lib_name)
                [~, ~] = loadlibrary(self.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end
            self.port_num = portHandler(self.DEVICENAME);
            packetHandler();
            if (openPort(self.port_num))
                fprintf('Succeeded to open the port!\n');
            else
                fprintf('Failed to open the port!\n');
                closePort(self.port_num);
                unloadlibrary(lib_name);
            end
            
            if (setBaudRate(self.port_num, self.BAUDRATE))
                fprintf('Succeeded to change the baudrate!\n');
            else
                unloadlibrary(self.lib_name);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end

            self.set_speed([0.1,0.1,0.1,0.1],true);
            self.set_torque_limit([1,1,1,1]);
            self.move_j(0,0,0,0);
            self.init_status = 1;
            
        end
        
        function open_gripper(self)
            if ~self.gripper_open_flag
                write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.gripper_motor_id, self.ADDR_MX_GOAL_POSITION, 0);
                self.gripper_open_flag = 1;
            end
        end
        
        function close_gripper(self)
            if self.gripper_open_flag
                write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.gripper_motor_id, self.ADDR_MX_GOAL_POSITION, 1023);
                self.gripper_open_flag = 0;
            end
        end
        
        function actuate_gripper(self)
            if self.gripper_open_flag
                self.close_gripper();
            else
                self.open_gripper();
            end
        end
        
        function smooth_speed(self,joint_angles)
            max_angle = max(abs(joint_angles));
            speed_per_deg = max_angle/100;
            if speed_per_deg~=0
                new_speeds = abs(joint_angles/speed_per_deg)*0.01;
                for i=1:length(self.motor_speed)
                    if new_speeds(i)==0
                        new_speeds(i)=self.motor_speed(i);
                    else
                        new_speeds(i)=new_speeds(i)*self.motor_speed(i);
                    end
                end
                self.set_speed(new_speeds,false);
            end
        end
        
        function create_rbt(self)
            self.rbt = rigidBodyTree;
            bodies = cell(4,1);
            joints = cell(4,1);
            for i = 1:4
                bodies{i} = rigidBody(['link' num2str(i)]);
                joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
                joints{i}.PositionLimits = [self.joint_limits(i,1)*pi/180,self.joint_limits(i,2)*pi/180];
                setFixedTransform(joints{i},self.dh(i,:),"dh");
                bodies{i}.Joint = joints{i};
                if i == 1 % Add first body to base
                    addBody(self.rbt,bodies{i},"base")
                else % Add current body to previous body by name
                    addBody(self.rbt,bodies{i},bodies{i-1}.Name)
                end
            end 
            self.ik = inverseKinematics('RigidBodyTree',self.rbt);
        end
        
        function set_speed(self, speeds, overwrite_speeds)
            if overwrite_speeds
                self.motor_speed = speeds;
            end
            for i=1:length(self.motor_ids)
                if speeds(i) > 0 && speeds(i) <= 1
                    speed = speeds(i)*1023;
                    write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), 32, speed);
                    dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
                    dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
                    if dxl_comm_result ~= self.COMM_SUCCESS
                        fprintf('\n%s', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                    elseif dxl_error ~= 0
                        fprintf('\n%s', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                    end
                else
                   fprintf("\nMovement speed out of range, enter value between ]0,1]"); 
                end
            end
        end
        
        function set_torque_limit(self, torques)
            self.motor_torque = torques;
            for i=1:length(self.motor_ids)
                if torques(i) > 0 && torques(i) <= 1
                    write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), 34, torques(i)*1023);
                    dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
                    dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
                    if dxl_comm_result ~= self.COMM_SUCCESS
                        fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                    elseif dxl_error ~= 0
                        fprintf('%s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                    end
                else
                   fprintf("\nTorque limit out of range, enter value between ]0,1]"); 
                end
            end
                end
        
        function enable_motors(self)         
            
            % Enable Dynamixel Torque
            for i=1:length(self.motor_ids)
                write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE);
                dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
                dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
                if dxl_comm_result ~= self.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                else
                    fprintf('\nDynamixel has been successfully connected, torque mode enabled \n');
                end
            end
            
        end
        
        function deg_present_position = get_position(self, motor_id)
            dxl_present_position = read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, motor_id, self.ADDR_MX_PRESENT_POSITION);
            deg_present_position = self.rot_to_deg(dxl_present_position);
            dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
            if dxl_comm_result ~= self.COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
            end
        end
        
        function disable_motors(self)
            
            for i=1:length(self.motor_ids)
                
                % Disable Dynamixel Torque
                write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE);
                dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
                dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
                if dxl_comm_result ~= self.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                else
                    fprintf('\nDynamixel succesfully disconnected');
                    
                end
            end
            % Close port
            closePort(self.port_num);
            
            % Unload Library
            unloadlibrary(self.lib_name);
            self.init_status = 0;
        end
        
        function deg = check_limits(self,deg, motor_id)
            % see https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
            if motor_id==self.motor_ids(1)
                assert(abs(deg) <= 130, "Angle Limits for first Axis Reached, Min/Max: +-130°");
            elseif motor_id==self.motor_ids(2)
                assert(deg <= 0 && deg >= -180, "Angle Limits for second Axis Reached, Min/Max: [0,-180]°");
            else
                assert(abs(deg) <= 100, "Angle Limits Reached, Min/Max: +-100°");
            end

        end
        
        function rot = deg_to_rot(self,deg)
            rot = deg*1/0.29;
        end
        
        function deg = rot_to_deg(self,rot)
            deg = rot*0.29;
        end
        
        function move_j(self,j1,j2,j3,j4)
            j1 = self.check_limits(j1, self.motor_ids(1));
            j2 = self.check_limits(j2, self.motor_ids(2));
            j3 = self.check_limits(j3, self.motor_ids(3));
            j4 = self.check_limits(j4, self.motor_ids(4));

            if self.use_smooth_speed_flag
                self.smooth_speed([j1 j2 j3 j4]-self.joint_angles)
            end
            self.joint_angles = [j1 j2 j3 j4];
            self.forward([j1 j2 j3 j4]);
            if self.draw_robot_flag
                self.draw_robot()
            end
            j1 = j1 + self.joint_offsets(1);
            j2 = j2 + self.joint_offsets(2);
            j3 = j3 + self.joint_offsets(3);
            j4 = j4 + self.joint_offsets(4);           

            
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(1), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j1));
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(2), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j2));
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(3), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j3));
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(4), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j4));
            
            while 1
                self.read_joint_angles();
                if self.joint_angle_error<2
                    break;
                end
            end
        end
        
        
        function draw_robot(self)
            if self.rbt == 0
                self.create_rbt();
            end
            
            
            config = homeConfiguration(self.rbt);
            for i=1:length(self.joint_angles)
                config(i).JointPosition = self.joint_angles(i)*pi/180;
            end
            if self.draw_robot_flag == 0
                figure(Name="RRRR Robot Model");
            end
            show(self.rbt,config);
            self.draw_robot_flag = 1;
        end
        
        function ee_cartesian_coords = forward(self, j_a)
           
            self.forward_transform = [cosd(j_a(1)) -sind(j_a(1))*cos(self.dh(1,2))  sind(j_a(1))*sin(self.dh(1,2)) self.dh(1,1)*cos(j_a(1));
                sind(j_a(1)) cosd(j_a(1))*cos(self.dh(1,2)) -cosd(j_a(1))*sin(self.dh(1,2)) self.dh(1,1)*sind(j_a(1));
                0 sin(self.dh(1,2)) cos(self.dh(1,2)) self.dh(1,3);
                0 0 0 1];
            
            self.joint_pos(:,1) = self.forward_transform * [0 0 0 1]' ;
            self.joint_pos(:,1) = self.joint_pos(:,1) / self.joint_pos(4,1);
            
            for i=2:length(j_a)
                self.forward_transform = self.forward_transform * [cosd(j_a(i)) -sind(j_a(i))*cos(self.dh(i,2))  sind(j_a(i))*sin(self.dh(i,2)) self.dh(i,1)*cosd(j_a(i));
                    sind(j_a(i)) cosd(j_a(i))*cos(self.dh(i,2)) -cosd(j_a(i))*sin(self.dh(i,2)) self.dh(i,1)*sind(j_a(i));
                    0 sin(self.dh(i,2)) cos(self.dh(i,2)) self.dh(i,3);
                    0 0 0 1];  
                self.joint_pos(:,i) = self.forward_transform * [0 0 0 1]' ;
                self.joint_pos(:,i) = self.joint_pos(:,i) / self.joint_pos(4,i);
            end
            
            ee_cartesian_coords = self.forward_transform * [0 0 0 1]' ;
            ee_cartesian_coords = ee_cartesian_coords/ee_cartesian_coords(4);
        end
        
        function j_a = inverse(self, x,y,z,pitch)
%             self.joint_angles(1) = atan2(y,x);
%             
%             r = sqrt(x^2+y^2);
%             d = sqrt(r^2+(z-0.091)^2);
%             h = sqrt(0.096^2-(d/2)^2);
%             self.joint_angles(2) = atan2(z-0.096,r) + atan2(h,d/2);
%             self.joint_angles(3) = -180 -2 * atan2(d/2,h);
%             self.joint_angles(4) = - self.joint_angles(2) - self.joint_angles(3);

%             self.joint_angles(1) = atan2(y,x);
%             self.joint_angles(2) = acos((x^2+y^2-self.dh(1,1)^2-self.dh(2,1)^2)/(2*self.dh(1,1)*self.dh(2,1)));
%             self.joint_angles(3) = acos(((sqrt(x^2+y^2)-self.dh(4,1))^2+(self.dh(1,3)-z)^2)/...
%                 (self.dh(2,1)^2+self.dh(3,1)^2 + 2*self.dh(2,1)*self.dh(3,1)));
%             self.joint_angles(4) = - self.joint_angles(2) - self.joint_angles(3);
        
        
%             a1 = 0;
%             a2 = 0.096;
%             a3 = 0.096;
%             a4 = 0.047;
%             d1 = 0.0919;
%             pitch = 0;
%             
% 
%             j1 = atan2(y,x);
%             j3 = pi + acos((a2^2/2 + a3^2/2 - (a4 - (x^2 + y^2)^(1/2))^2/2 - (d1 - z)^2/2)/(a2*a3));
%             %j3 = pi - acos((a2^2/2 + a3^2/2 - (a4 - (x^2 + y^2)^(1/2))^2/2 - (d1 - z)^2/2)/(a2*a3));
%             
%             j2 = (pi-j3)/2 + acos(sqrt((x^2+y^2)/(z^2+x^2+y^2)));
%             j4 = pitch - j2 - j3;

% 
%             if self.rbt == 0
%                 self.create_rbt();
%             end
%             
%             j_a = zeros(4,1);            
%             initialguess = self.rbt.homeConfiguration;           
%             tform = [ 1 0 0 x;
%                 0 0 1 y;
%                 0 -1 0 z;
%                 0 0 0 1];
%             
%             [configSoln,solnInfo] = self.ik('link4',tform,self.ik_weights,initialguess);
%             if strcmp(solnInfo.Status,'success')
%                 for i=1:4
%                     j_a(i) = configSoln(i).JointPosition*180/pi;
%                 end
%             elseif strcmp(solnInfo.Status,'best available')
%                 fprintf("Status: '%s', using Angles:\n", solnInfo.Status);
%                 for i=1:4
%                     j_a(i) = configSoln(i).JointPosition*180/pi;
%                     fprintf("%f°  ",configSoln(i).JointPosition*180/pi);
%                 end
%             else
%                 fprintf("Unsuccesfull IK, with status: '%s'", solnInfo.Status);
%             end
        
            j1 = atan2(y,x);
            j3 = acos( ((sqrt(x^2+y^2)-self.dh(4,1)*cos(pitch))^2 + (self.dh(1,3)-z)^2 - self.dh(2,1)^2 - self.dh(3,1)^2) / (2*self.dh(2,1)*self.dh(3,1)) );
            assert(isreal(j3),"Configuration Impossible");
            j2 = -atan2(z-self.dh(1,3)-self.dh(4,1)*sin(pitch),sqrt(x^2+y^2)-self.dh(4,1)*cos(pitch)) - atan2(self.dh(3,1)+self.dh(2,1)*cos(j3),self.dh(2,1)*sin(j3)) + (pi/2-j3);
            j4 = pitch - j2 - j3;
            
            j_a = rad2deg([j1 j2 j3 j4]);
            self.pitch = pitch;
            assert(isreal(j_a),"Configuration Impossible");
        end
        
        function j_a = read_joint_angles(self)
            j_a = zeros(4,1);
            for i=1:length(self.motor_ids)
                dxl_present_position = read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), 36);
                dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
                dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
                if dxl_comm_result ~= self.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                else
                    j_a(i) = self.rot_to_deg(dxl_present_position) - self.joint_offsets(i);
                    self.joint_angle_error(i) = j_a(i)-self.joint_angles(i);
                end 
            end
        end
        
        function ee_pos = read_ee_position(self)
           j_a = self.read_joint_angles();
           ee_pos = self.forward(j_a);
        end
        
        function move_c (self,x,y,z,pitch)
           j_a = self.inverse(x,y,z,deg2rad(pitch));
           self.move_j(j_a(1),j_a(2),j_a(3),j_a(4));
        end
        
        function record_configuration(self)
            j_a = self.joint_angles;
            torque = self.motor_torque(1);
            speed = self.motor_speed(1);
            if isempty(self.movement_history)
                self.movement_history = [j_a(1), j_a(2),j_a(3),j_a(4), speed, torque,self.gripper_open_flag];
            else
                self.movement_history = [self.movement_history; j_a(1), j_a(2),j_a(3),j_a(4), speed, torque,self.gripper_open_flag];
            end
            fprintf("\nRecorded Speed: %f, Torque: %f, \nJoint Positions: %f, %f, %f, %f,\nGripper open: %f",speed,torque,j_a(1),j_a(2),j_a(3),j_a(4), self.gripper_open_flag);
        end
        
        function delete_last_recorded_configuration(self)
            length_history = size(self.movement_history);
            if isempty(self.movement_history)
                fprintf("No last history position"); 
            elseif length_history(1)==1
                self.movement_history = [];
            else
               self.movement_history(end,:) = [];
            end
        end
        
        function play_configuration_history(self)
            if ~isempty(self.movement_history)
               length_history = size(self.movement_history);
               for i=1:length_history(1)
                  speed = self.movement_history(i,5);
                  torque = self.movement_history(i,6);
                  self.set_speed([speed,speed, speed, speed],true);
                  self.set_torque_limit([torque, torque, torque, torque]);
                  self.move_j(self.movement_history(i,1),self.movement_history(i,2),self.movement_history(i,3),self.movement_history(i,4));
                  pause(1);
                  if self.gripper_open_flag ~= self.movement_history(i,7)
                      self.actuate_gripper();
                      pause(3);
                  end
               end
            end
        end

    end
end



