# mpc-ball-control
This project as aim to control a ball on a plate using a Sawyer robotic manipulator, a camera and an MPC controller.
You should be able to reproduce our work with any other manipulator or system as long as you car control the pitch/roll
and change the joint control

## Reproduce this work
1. Create a catkin workspace
```
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

2. Clone this repository
```
cd ~/catkin_ws/src/
git clone https://github.com/JulienRineau/mpc-ball-control.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

3. Launch the ball tracker
```
rosrun tracking circle_detector.py
```

4. Calibrate the camera
TODO

3. Launch the ball tracker
TODO
```
rosrun joint_control osqp..
```
