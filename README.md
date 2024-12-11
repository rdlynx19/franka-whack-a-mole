# Franka Whack-a-mole ME 495 Final Project
Authors: Ben Benyamin, David Khachatryan, Haodong Wang, Pushkar Dave, Sairam Umakanth

## Description
This package contains Python API for making a Franka Emika Panda robot play whack-a-mole. The 
physical setup requires a RealSense2 camera, a singular April tag attached to the base of the 
robot, a servo/hammer end-effector attachment connected to an Arduino microcontroller, and 
optionally a DIY whack-a-mole set attached to its own Arduino microcontroller. The game is played
using color and illumination detection and the "moles" are hit using a servo-hammer end-effector
attachment, the IDE code for which is also provided within this repository. The franka robot is
operated using MoveIt wrapper API created by our team 
(https://github.com/ME495-EmbeddedSystems/homework-3-group-4) and to use whack-a-mole API, 
the Moveit wrapper API must be cloned in the same workspace as the whack-a-mole API. The 
package also contains Arduino IDE code to program a DIY whack-a-mole. 

## QuickStart
1. Set up a `workspace` on your computer with the `src` directory.
2. Go to your `workspace/src` and clone this repository.
3. In the same `workspace/src`, clone our MoveItAPI (https://github.com/ME495-EmbeddedSystems/homework-3-group-4) and `colcon build` all repositories
4. [OPTIONAL] Create servo/hammer end-effector attachment and upload `servo_serial_communication.ino` to it via an Arduino microcontroller
5. [OPTIONAL] Create DIY whack-a-mole set and `build_moles.ino`to it via an Arduino microcontroller 
6. Set up April Tag 36h11:0 with base attachment found here (https://github.com/wengmister/Apex-Putter/issues/2)
7. Mount Realsense 2 camera in a location that is never impeded by the arm and `sudo install` requied `realsense2` and `apriltag_ros` packages
8. Connect to Franka Emika Panda Robot
9. Run  `ros2 launch whack_a_mole planner_swing.launch.xml` in terminal to load Rviz configuration of physical set-up
10. [OPTIONAL] Use `ros2 service call /go_home std_srvs/srv/Empty` to send robot to home configuration
11. Run `ros2 service call /call_play std_srvs/srv/Empty` to make the robot play the game and enjoy!

## Nodes and Launchfiles
### Nodes:
- `comm_node`: interfaces between other ROS nodes and servo/hammer for actuation of hammer
- `swing`: sends path planning and execution commands to MoveIt API
- `camera_node`: uses realsense2 topics for color and illumination detection and broadcasts tf of colors 
- `arduino_hint`: receives feedback from whack-a-mole and loops call of `/play` service in `call_play` callback to play game in perpetuality
- `game`: looks up transforms of colors and sends service request to swing to actuate hammer

### Launchfiles:
- `camera.launch.py`: launches `camera_node` and `realsense2_camera` for color detection, and also launches rviz files for physical setup
- `planner_swing.launch.xml`: launches rest of nodes created in package, `apriltag_node` required for april tag detection, and includes `camra.launch.py` and `object_mover.launch.py` in `object_mover`, the MoveIt API package

## System Architecture 

![rqt_graph](https://github.com/user-attachments/assets/d69f05cb-da2e-4601-9f59-a3a448614af6)

![tf2_view_frames](https://github.com/user-attachments/assets/47dc3262-8e01-40a6-a1b8-ad64a84bdee5)

## Video Demonstrations
### Rviz 

[whack-a-mole_live.webm](https://github.com/user-attachments/assets/87ae88f5-a53d-4895-9542-463258fc8a94)

### Physical Demo

[whack-a-mole_live.webm](https://drive.google.com/file/d/19mdgvS2EuuI-zHf7KM9rPcOu6m58NArn/view?usp=drive_link)
