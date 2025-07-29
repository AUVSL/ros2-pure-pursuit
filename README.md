# ros2-pure-pursuit
A python RGator pure pursuit controller written primarily by Jiaming Zhang.

## Installation
1. Make Workspace
```
cd
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
2. Install Code and Custom ROS Message Formats
```
cd
pip install pandas
pip install numpy==1.23.0
sudo apt install python3-can
git clone https://github.com/AUVSL/rgator_computer_ws.git
git clone -b ros2 https://github.com/dawonn/vectornav.git
git clone https://github.com/AUVSL/ros2-pure-pursuit.git
```
3. Compile Code and Messages
```
cd ..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r
colcon build
```

## Package Breakdown:

  - track_log: log x,y,heading data from imu, convert to .csv file
  - path_visual: visualize track
  - pure_pur: pure pursuit for steering control with a lookahead in front of the vehicle


## Running Scripts:
### Path track

    ros2 launch vectornav vectornav.launch.py
    ros2 run dbw dbw
    ros2 run ros2-pure-pursuit pure_pur

### Path log

    ros2 launch vectornav vectornav.launch.py
    ros2 run ros2-pure-pursuit track_log
