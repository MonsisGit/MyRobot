<p float="left">
  <img src="https://github.com/MonsisGit/MyRobot/blob/master/doc/images/dtu.png" width="60" height="80" /> 
</p>

# MyRobot
This robot was developed by me during an internship at Novo Nordisk in collaboration with DTU. It is intended to be fully open source. For inquiries, ideas or collaboration feel free to message me at _simon.goldhofer@gmail.com_ .

## Links

0. [Setup](https://github.com/MonsisGit/MyRobot/blob/master/README.md#Setup)
1. [Theory](https://github.com/MonsisGit/MyRobot/blob/master/doc/Programming_of_robotic_arm_with_computer_vision_based_control.pdf)
2. Code Usage 
    - [Basic MyRobot class usage (MATLAB)](https://github.com/MonsisGit/MyRobot/blob/master/README.md#Basic-Usage-of-MyRobot)
    - [Visual Servoing Demo (MATLAB)](https://github.com/MonsisGit/MyRobot/blob/master/README.md#Visual-Servoing)
    - [Controlling the arm via GUI (MATLAB)](https://github.com/MonsisGit/MyRobot/blob/master/README.md#Using-the-GUI)

3. [CAD](https://github.com/MonsisGit/MyRobot/blob/master/CAD/) includes:
    - [CAD Files (STP)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/Robot_Arm_stp.zip)
    - [CAD Files (CATIA V5)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/CATIA%20V5.zip)
    - [Parts list (xlsx)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/parts_list.xlsx)
    - [Parts list (pdf)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/parts_list.pdf)
    - [STL Files for 3D-Printing](https://github.com/MonsisGit/MyRobot/blob/master/CAD/stls_to_print.zip)
    - [Drawing](https://github.com/MonsisGit/MyRobot/blob/master/CAD/overview_drawing.pdf)

## Setup
1. Run ```git pull https://github.com/MonsisGit/MyRobot.git```
2. Follow the [setup instructions](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/matlab_windows/) for the Dynamixel SDK in matlab. The repo includes [Dynamixel SDK-3.7.31](https://github.com/MonsisGit/MyRobot/tree/master/DynamixelSDK-3.7.31), change paths accordingly.
3. Optional: Install the [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) and [configure](https://docs.poppy-project.org/en/assembly-guides/poppy-humanoid/addressing_dynamixel.html) your motors from there (e.g. Motor IDs).
4. Optional: For [DirectServo](https://github.com/MonsisGit/MyRobot/blob/master/doc/DirectServo.md) setup robot with a USB-camera. Run ```webcamlist()``` to find the new camera.
5. [Pictures](https://github.com/MonsisGit/MyRobot/tree/master/doc/images) of the arm may assist with mechanical setup.
## Basic Usage of MyRobot
See documentaion of MyRobot Class
```
doc MyRobot
```

Initialize the robot, it should move to home configuration (0°,0°,0°,0°)
```
robot = MyRobot();
```
This should return
```
Dynamixel succesfully disconnected
Dynamixel succesfully disconnected
Dynamixel succesfully disconnected
Dynamixel succesfully disconnected
Succeeded to open the port!
Succeeded to change the baudrate!
```
Set movements speed of each individual joint, update interal joint speeds for later commands.
```
robot.set_speed([0.1,0.1,0.1,0.2],true);
```

Set all motors to maximum torque.
```
robot.set_torque_limit([1,1,1,1]);
```

Draw the current configuration of the robot. This get updated with every new movement command issued.
```
robot.draw_robot();
```

Move the robots joints.
```
robot.move_j(20,-90,0,50);
```
Move using inverse kinematics. Input are cartesian x, y, z position of the end-effector in meters. The last input argument is the pitch angle, which refers to the angle in degrees between the end-effector and the horizontal. The second command lies outside the robots workspace, the Error *Configuration Impossible* gets thrown. The robot does not move.
```
rob.move_c(0.05,0.12,0.25,0);
rob.move_c(0.05,0.12,0.35,0);

Error using MyRobot/inverse (line 512)
Configuration Impossible

Error in MyRobot/move_c (line 574)
           j_a = self.inverse(x,y,z,deg2rad(pitch));
```

Actuate the gripper. If the gripper is currently closed, it will open.
```
robot.actuate_gripper();
robot.close_gripper();
robot.open_gripper();
```

Get the robots current joint positions.
```
current_joint_positions = robot.joint_pos
```
This returns joint positions 1-4 (columns) and x,y,z (rows) in meters. For reference see the [drawing](https://github.com/MonsisGit/MyRobot/blob/master/CAD/overview_drawing.pdf). Here, the initial position is shown:
```
ans =
         0    0.1160    0.2120    0.3081
         0         0         0         0
    0.0955    0.0955    0.0955    0.0955
    1.0000    1.0000    1.0000    1.0000
```
Disable all motors. This is necessary to free up the com port. If you forgot to do this and clear the robot object, it will fail at reinitialization. To fix this unplug the robots USB cable and clear the workspace.
```
robot.disable_motors();
```
This should return
```
Dynamixel succesfully disconnected
Dynamixel succesfully disconnected
Dynamixel succesfully disconnected
Dynamixel succesfully disconnected
```

## Visual Servoing

To run, the deep learning toolbox and the MATLAB support package for USB webcams are needed.

Run 
```
webcamlist()

ans =

  4×1 cell array

    {'GENERAL WEBCAM'         }
    {'HP TrueVision HD Camera'}
    {'C270 HD WEBCAM'         }
    {'DroidCam Source 3'      }
```
to define correct camera
```
cam = webcam(3);
```
Demo:


<img src="https://media.giphy.com/media/OC3qdhHCzLoz7IeaYO/giphy.gif" width="500" height="300" />

## Using the GUI

Install [app](https://github.com/MonsisGit/MyRobot/blob/master/matlab/MyRobot_Studio.mlappinstall) or run [file](https://github.com/MonsisGit/MyRobot/blob/master/matlab/MyRobot_Studio.mlapp)
![GUI](https://github.com/MonsisGit/MyRobot/blob/master/doc/images/gui.png)
Demo:


<img src="https://media.giphy.com/media/SqHr7QHvoKA8Fz5jO6/giphy.gif" width="500" height="300" />

## Contact
Simon Goldhofer: simon.goldhofer@gmail.com

