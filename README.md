# swarm_pso (ongoing)

## Installation and Setup
Download the PX4-Autopilot repository and build it outside your catkin workspaces (if any)
```
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh
make px4_sitl gazebo_classic
```
This will launch a gazebo window with the empty world and iris drone model (Press Ctrl+C in terminal to close the seesion)

> If an error is encountered regarding test_data in PX4-Autopilot then delete the `test_data` folder from the PX4-Autopilot folder and retry the last command

#### Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu) 

Create a catkin workspace, clone our repo and then build it with catkin
```
cd
mkdir swarm_ws
mkdir swarm_ws/src
cd swarm_ws
catkin_init_workspace
cd src
git clone https://github.com/sohamsarpotdar2001/swarm_pso.git
cd ..
catkin build
```

## Making all the files executable
```
cd swarm_ws/src/swarm_pso/launch
chmod +x *
cd swarm_ws/src/swarm_pso/src
chmod +x *
```

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
