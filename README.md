 | 
:-------------------------:|:-------------------------:
<div style="width:100px ; height:60px">
![](https://github.com/MonsisGit/MyRobot/blob/master/doc/images/novo.png)
<div>  |  <div style="width:100px ; height:60px">
![](https://github.com/MonsisGit/MyRobot/blob/master/doc/images/dtu.png)
<div>


# MyRobot
This robot was developed by me during an internship at Novo Nordisk in collaboration with DTU

## Links

1. [Theory](https://github.com/MonsisGit/MyRobot/blob/master/doc/theory.md)
2. Code Usage 
    - [MyRobot](https://github.com/MonsisGit/MyRobot/blob/master/doc/MyRobot.md)
    - [DirectServo](https://github.com/MonsisGit/MyRobot/blob/master/doc/DirectServo.md)
    - [GUI](https://github.com/MonsisGit/MyRobot/blob/master/doc/GUI.md)

3. CAD
    - [Complete CAD File (STP)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/Robot_Arm.stp)
    - [Complete CAD File (CATIA)](https://github.com/MonsisGit/MyRobot/blob/master/CAD/Robot_Arm.stp)
    - [Parts list](https://github.com/MonsisGit/MyRobot/blob/master/CAD/parts_list.md)
    - [STL Files](https://github.com/MonsisGit/MyRobot/blob/master/CAD/stl/)

## Basic Usage

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

##Using the GUI
Insert video


