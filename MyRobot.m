classdef MyRobot < handle
    properties
        lib_name = 'dxl_x64_c';
        %if ~libisloaded(lib_name)
        %    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
        %end
        
        % Control table address
        ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model
        ADDR_MX_GOAL_POSITION       = 30;
        ADDR_MX_PRESENT_POSITION    = 36;
        
        % Protocol version
        PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

        motor_ids = [2 1 10 0];
        
        BAUDRATE                    = 1000000;
        DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
        % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
        
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold
        
        ESC_CHARACTER               = 'e';          % Key for escaping loop
        
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        
        index = 1;
        port_num=0;
        
        % a, alpha, d, theta
        dh = [0   	-pi/2	0.0919 0;
            0.096	0       0       0;
            0.096	0	0	0;
            0.047  	0	0	0];
        
       
        A = zeros(4,4);
        
        joint_angles = [0 0 0 0];
        joint_pos = zeros(4,4);
        
        draw_robot_flag = 0;
        rbt = rigidBodyTree;
        
        joint_limits = [-180 180; -160 0; -80 80; -80 80];
        
        ik = 0;
        ik_weights = [0.25 0.25 0.25 1 1 1];
        joint_offsets = [171 150+90 150 150];
        joint_angle_error = [0 0 0 0];
   
    end
    methods
        function self = MyRobot()
            if ~libisloaded(self.lib_name)
                [notfound, warnings] = loadlibrary(self.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end
            self.port_num = portHandler(self.DEVICENAME);
            packetHandler();
            % Open port
            if (openPort(self.port_num))
                fprintf('Succeeded to open the port!\n');
            else
                fprintf('Failed to open the port!\n');
                % Close port
                closePort(self.port_num);
                unloadlibrary(lib_name);


            end
            
            % Set port baudrate
            if (setBaudRate(self.port_num, self.BAUDRATE))
                fprintf('Succeeded to change the baudrate!\n');
            else
                unloadlibrary(self.lib_name);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end
            fprintf("Initial Joint Angles: ")
            for i=1:length(self.motor_ids)
                self.joint_angles(i) = get_position(self, self.motor_ids(i));
                fprintf('\n[ID:%03d] Pos:%02d', self.motor_ids(i), self.joint_angles(i));

            end
            self.set_speed([0.5,0.3,0.3,0.3]);
            fprintf("\nSet initial speed to [05, 0.3, 0.3, 0.3]");
            self.set_torque_limit([1,1,1,1]);
            fprintf("\nSet initial torque limit to [1 1 1 1]");
            self.move_j(0,0,0,0);
            self.create_rbt();
            
        end
        
        function create_rbt(self)
            bodies = cell(4,1);
            joints = cell(4,1);
            for i = 1:4
                bodies{i} = rigidBody(['body' num2str(i)]);
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
        
        function set_speed(self, speeds)
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
                   fprint("\nMovement speed out of range, enter value between ]0,1]"); 
                end
            end
        end
        
        function set_torque_limit(self, torque)
            for i=1:length(self.motor_ids)
                if torque(i) > 0 && torque(i) <= 1
                    tor = torque(i)*1023;
                    write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), 34, tor);
                    dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
                    dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
                    if dxl_comm_result ~= self.COMM_SUCCESS
                        fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                    elseif dxl_error ~= 0
                        fprintf('%s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                    end
                else
                   fprint("\nTorque limit out of range, enter value between ]0,1]"); 
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
        end
        
        function deg = check_limits(self,deg, motor_id)
            % see https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
            if motor_id==self.motor_ids(1)
                assert(deg <= 150 && deg >= -150, "Angle Limits for first Axis Reached, Min/Max: +-90°");
            elseif motor_id==self.motor_ids(2)
                assert(deg <= 0 && deg >= -160, "Angle Limits for second Axis Reached, Min/Max: +-90°");
            else
                assert(abs(deg) <= 80, "Angle Limits Reached, Min/Max: +-90°");
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

            self.joint_angles = [j1 j2 j3 0];
            self.forward([j1 j2 j3 0]);
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
            pause(1)
        end
        
        
        function draw_robot(self)
            self.draw_robot_flag = 1;
            
            config = homeConfiguration(self.rbt);
            for i=1:length(self.joint_angles)
                config(i).JointPosition = self.joint_angles(i)*pi/180;
            end
            figure(Name="RRRR Robot Model");
            show(self.rbt,config);
        end
        
        function ee_cartesian_coords = forward(self, j_a)
           
            self.A = [cosd(j_a(1)) -sind(j_a(1))*cos(self.dh(1,2))  sind(j_a(1))*sin(self.dh(1,2)) self.dh(1,1)*cos(j_a(1));
                sind(j_a(1)) cosd(j_a(1))*cos(self.dh(1,2)) -cosd(j_a(1))*sin(self.dh(1,2)) self.dh(1,1)*sind(j_a(1));
                0 sin(self.dh(1,2)) cos(self.dh(1,2)) self.dh(1,3);
                0 0 0 1];
            
            self.joint_pos(:,1) = self.A * [0 0 0 1]' ;
            self.joint_pos(:,1) = self.joint_pos(:,1) / self.joint_pos(4,1);
            
            for i=2:length(j_a)
                self.A = self.A * [cosd(j_a(i)) -sind(j_a(i))*cos(self.dh(i,2))  sind(j_a(i))*sin(self.dh(i,2)) self.dh(i,1)*cosd(j_a(i));
                    sind(j_a(i)) cosd(j_a(i))*cos(self.dh(i,2)) -cosd(j_a(i))*sin(self.dh(i,2)) self.dh(i,1)*sind(j_a(i));
                    0 sin(self.dh(i,2)) cos(self.dh(i,2)) self.dh(i,3);
                    0 0 0 1];  
                self.joint_pos(:,i) = self.A * [0 0 0 1]' ;
                self.joint_pos(:,i) = self.joint_pos(:,i) / self.joint_pos(4,i);
            end
            
            ee_cartesian_coords = self.A * [0 0 0 1]' ;
            ee_cartesian_coords = ee_cartesian_coords/ee_cartesian_coords(4);
        end
        
        function j_a = inverse(self, x,y,z)
%             self.joint_angles(1) = atan2(y,x);
%             
%             r = sqrt(x^2+y^2);
%             d = sqrt(r^2+(z-0.091)^2);
%             h = sqrt(0.096^2-(d/2)^2);
%             self.joint_angles(2) = atan2(z-0.096,r) + atan2(h,d/2);
%             self.joint_angles(3) = -180 -2 * atan2(d/2,h);
%             self.joint_angles(4) = - self.joint_angles(2) - self.joint_angles(3);


            j_a = zeros(4,1);            
            initialguess = self.rbt.homeConfiguration;           
            tform = [ 1 0 0 x;
                0 0 1 y;
                0 -1 0 z;
                0 0 0 1];
            
            [configSoln,solnInfo] = self.ik('body4',tform,self.ik_weights,initialguess);
            if strcmp(solnInfo.Status,'success')
                for i=1:4
                    j_a(i) = configSoln(i).JointPosition*180/pi;
                end
            elseif strcmp(solnInfo.Status,'best available')
                fprintf("Status: '%s', using Angles:\n", solnInfo.Status);
                for i=1:4
                    j_a(i) = configSoln(i).JointPosition*180/pi;
                    fprintf("%f°  ",configSoln(i).JointPosition*180/pi);
                end
            else
                fprintf("Unsuccesfull IK, with status: '%s'", solnInfo.Status);
            end
               
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
        
        function move_c (self,x,y,z)
           j_a = self.inverse(x,y,z);
           self.move_j(j_a(1),j_a(2),j_a(3),j_a(4));
        end

    end
end



