# Franka Whack-a-mole ME 495 Final Project
Authors: Ben Benyamin, David Khachatryan, Haodong Wang, Pushkar Dave, Sairam Umakanth

## Demo

Note: This video is lengthy and could not be embedded directly into the README. Please click on the image below to view the video.

[![Video Title](https://img.youtube.com/vi/GG81r0z8W5s/0.jpg)](https://youtu.be/GG81r0z8W5s?si=jnvBzOhuwIO57tmf)

## Description
This package enables the Franka Emika Panda robot play whack-a-mole on a custom whack-a-mole game. The 
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
1. Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
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
### Architecture Description:

The `camera_node` subcsribes topics published by the realsense camera and detects colors and broadcasting the frames of the colors to the TF tree, thus making the camera and color frames visible on Rvizz. To bring in the robot, the MoveIt API is launched along with the `apriltag_node` and `game`. The `apriltag_node` detects the april tag attached to the base of the robot. In the `game` node, a static transform between the base of the robot and the base april tag and the transform is computed using real world values once the april tag is attached to the base of the robot. Now the tf tree has all the info it needs for rviz to map the entire physical setup. Once the `/call_play` service is called, it causes a loop of `/play` service calls within the `hint` node. This `hint` node also receives a stream of data from the arduino on which button is lit up. Within the `/play` service callback in the `game` node, the transforms for the illuminated color is looked up from the tf tree and the `/pick_pose` custom service type is called, for which the service is located within the `comm_node`. Within the `pick_pose` callback, it interacts with MoveIt API to plan a path to the request goal pose and completes the action of swinging the hammer by calling custom action type called `swing_hammer`. The `swing_hammer` action sends a request to the servo controlled by an Arduino microcontroller to actuate the servo and swing the hammer, thus completing the hit action. This process is looped in `hint` node until the user kills the node on the terminal. The associated RQT graph and TF tree are shown below.

### RQT Graph:

![rqt_graph](https://github.com/user-attachments/assets/d69f05cb-da2e-4601-9f59-a3a448614af6)

### TF Tree:

![tf2_view_frames](https://github.com/user-attachments/assets/de01adff-58a2-418e-9448-719bc31216c0)


## Video Demonstrations
### Rviz 
[whack-a-mole_rvizz.webm](https://github.com/user-attachments/assets/669d5b71-3139-480c-ba2a-61dcc3d9fc77)

