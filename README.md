<p float="left">
  <img src="https://github.com/MonsisGit/MyRobot/blob/master/doc/images/novo.png" width="160" height="120" />
  <img src="https://github.com/MonsisGit/MyRobot/blob/master/doc/images/dtu.png" width="70" height="70" /> 
</p>

# MyRobot
This robot was developed by me during an internship at Novo Nordisk in collaboration with DTU. It is intended to be fully open source. For inquiries, ideas or collaboration feel free to message me at _simon.goldhofer@gmail.com_ .

## Links

1. [Theory](https://github.com/MonsisGit/MyRobot/blob/master/doc/theory.md)
2. Code Usage 
    - [Basic MyRobot class usage (MATLAB)](https://github.com/MonsisGit/MyRobot/blob/master/README.md#Basic-Usage-of-MyRobot)
    - [Visual Servoing Demo (MATLAB)](https://github.com/MonsisGit/MyRobot/blob/master/README.md#Visual-Servoing)
    - [DirectServo](https://github.com/MonsisGit/MyRobot/blob/master/doc/DirectServo.md)
    - [Controlling the arm via GUI (MATLAB)](https://github.com/MonsisGit/MyRobot/blob/master/README.md#Using-the-GUI)

3. [CAD](https://github.com/MonsisGit/MyRobot/blob/master/CAD/) includes:
    - [CAD Files (STP)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/Robot_Arm_stp.zip)
    - [CAD Files (CATIA V5)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/CATIA%20V5.zip)
    - [Parts list (xlsx)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/parts_list.xlsx)
    - [Parts list (pdf)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/parts_list.pdf)
    - [STL Files for 3D-Printing](https://github.com/MonsisGit/MyRobot/blob/master/CAD/stls_to_print.zip)
    - [Drawing](https://github.com/MonsisGit/MyRobot/blob/master/CAD/overview_drawing.pdf)

## Basic Usage of MyRobot
See documentaion of MyRobot Class
```
doc MyRobot
```

Initialize the robot, it should move to home configuration (0°,0°,0°,0°)
```
robot = MyRobot();
```

Set movements speed of each individual joint, update interal joint speeds for later commands
```
robot.set_speed([0.1,0.1,0.1,0.2],true);
```

Set all motors to maximum torque
```
robot.set_torque_limit([1,1,1,1]);
```

Draw the current configuration of the robot
```
robot.draw_robot();
```

Move the robots joints
```
robot.move_j(20,-90,0,50);
```

Actuate the gripper. If the gripper is currently closed, it will open
```
robot.actuate_gripper();
```

Get the robots current joint positions
```
current_joint_positions = robot.joint_pos
```

Disable all motors. This is necessary to free up the com port. If you forgot to do this and clear the robot object, it will fail at reinitialization. To fix this unplug the robots USB cable and clear the workspace
```
robot.disable_motors();
```
##Visual Servoing

To run, the deep learning toolbox and the MATLAB support package for USB webcams are needed


## Using the GUI

Start the MyRobot_Studio.mlapp file in Matlab

<img src="https://media.giphy.com/media/SqHr7QHvoKA8Fz5jO6/giphy.gif" width="700" height="500" />

