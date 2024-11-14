# swarm_pso (ongoing)

## Installation and Setup
Assuming ROS Noetic is installed. If not, [here](https://wiki.ros.org/noetic/Installation/Ubuntu) are the steps to do so.   \
Download the PX4-Autopilot repository and build it outside your catkin workspaces (if any)
```
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
```
Restart the computer and then try in a new terminal
```
cd PX4-Autopilot
make px4_sitl gazebo-classic
```
This will launch a gazebo window with the empty world and iris drone model (Press Ctrl+C in terminal to close the seesion)

> If an error is encountered regarding test_data in PX4-Autopilot then delete the `test_data` folder from the PX4-Autopilot folder and retry the last command

#### Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu) 

Create a catkin workspace, clone our repo and then build it with catkin
```
mkdir -p ~/swarm_ws/src
cd ~/swarm_ws
catkin init
cd src
git clone https://github.com/sohamsarpotdar2001/swarm_pso.git
cd ..
catkin build
```

## Making all the files executable
```
cd ~/swarm_ws/src/swarm_pso/launch
chmod +x *
cd ~/swarm_ws/src/swarm_pso/src
chmod +x *
```

## Installing Mavros
```
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## Editing .bashrc to source workspaces
```
cd
nano .bashrc
```
Go to the end of the file and add these lines
```
source ~/swarm_ws/devel/setup.bash
export px4_dir=~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/swarm_ws/src/swarm_pso/models
```
Press Ctrl+O and ENTER to save the file and Ctrl+X to exit  \
Close the terminal

## Launching the drone swarm and taking off
In one terminal window -
```
roslaunch swarm_pso multi_uav_mavros_sitl.launch
```
In other terminal window-
```
roslaunch swarm_pso drones_control.launch
```

## Error debugging
- After launching the `drones_control.launch` file if you see an error in the px4 terminal which is something like this `ERROR [simulator] poll timeout 0, 22` then just ignore it and wait until you see OFFBOARD enabled for all 10 drones.
- After that point the error should not appear and all the drones should arm successfully and takeoff in the gazebo environment. If at this point you don't see the drones taking off and a warning in the px4 terminal as `Arming denied: Resolve system health failures first` then Open a new terminal and
  - Locate the folder where you have installed QGroundControl
  - Run it by `./QGroundControl.AppImage`
  - After the QGC window loads, keep pressing OK to accept the connections of all the 10 drones
  - Click on the Top left corner where it is written as `Offboard` beside the logo
  - You will see some errors related to some parameters whose value need to be changed. Usually they are related to `COM_RC_IN_MODE` and `COM_OBS_AVOID`.
  - Change the `COM_OBS_AVOID` to Disabled and `COM_RC_IN_MODE` to stick input disabled 

## Running pso
After launching drones_control.launch file and confirming that all drones are taking off
```
roslaunch swarm_pso pso_search.launch
```
In other terminal, you can run the latency monitor to calulate latencies
```
rosrun swarm_pso latency_monitor.py
```
