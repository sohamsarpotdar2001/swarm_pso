#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import String
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamGet, ParamSet
import threading

class swarm_control:
    def __init__(self):
        self.no_of_drones = 10
        rospy.init_node("swarm_control", anonymous=True)
        self.rate = rospy.Rate(10)
        
        # Initialize state variables
        self.states = [None] * self.no_of_drones
        self.extended_states = [None] * self.no_of_drones
        self.curr_positions = [None] * self.no_of_drones
        self.pose_updated = [False] * self.no_of_drones
        
        # Initialize service clients and publishers
        self.pos_publishers = []
        self.arming_clients = []
        self.mode_clients = []
        self.param_get_clients = []
        self.param_set_clients = []
        self.state_subscribers = []
        self.extended_state_subscribers = []
        self.position_subscribers = []
        
        # Initialize message storage
        for i in range(self.no_of_drones):
            globals()[f"state_{i}"] = State()
            globals()[f"extended_state_{i}"] = ExtendedState()
            globals()[f"pos{i}"] = PoseStamped()
        
        self.arm_msg_pub = rospy.Publisher("/uavs/arm_msg", String, queue_size=10)
    
    def create_extended_state_callback(self, drone_id):
        def callback(data):
            try:
                extended_state_var = globals()[f"extended_state_{drone_id}"]
                extended_state_var.vtol_state = data.vtol_state
                extended_state_var.landed_state = data.landed_state
                self.extended_states[drone_id] = extended_state_var
            except Exception as e:
                rospy.logerr(f"Error in extended state callback for drone {drone_id}: {str(e)}")
        return callback

    def create_callback(self, drone_id):
        def callback(data):
            try:
                state_var = globals()[f"state_{drone_id}"]
                state_var.connected = data.connected
                state_var.armed = data.armed
                state_var.guided = data.guided
                state_var.mode = data.mode
                state_var.system_status = data.system_status
                self.states[drone_id] = state_var
                # Debug output for state changes
                rospy.logdebug(f"UAV{drone_id} state updated - Armed: {data.armed}, Mode: {data.mode}")
            except Exception as e:
                rospy.logerr(f"Error in state callback for drone {drone_id}: {str(e)}")
        return callback

    def create_pos_callback(self, drone_id):
        def callback(data):
            try:
                if self.curr_positions[drone_id] is None:
                    self.curr_positions[drone_id] = Point()
                
                self.curr_positions[drone_id].x = data.position.x
                self.curr_positions[drone_id].y = data.position.y
                self.curr_positions[drone_id].z = data.position.z
                self.pose_updated[drone_id] = True
                
            except Exception as e:
                rospy.logerr(f"Error in position callback for drone {drone_id}: {str(e)}")
        return callback

    def verify_drone_state(self, drone_id):
        # """Verify that a drone's state is being properly updated"""
        if self.states[drone_id] is None:
            rospy.logwarn(f"UAV{drone_id} state is None. Attempting to reinitialize...")
            # Attempt to reinitialize the subscriber
            self.initialize_single_drone(drone_id)
            return False
        return True

    def initialize_single_drone(self, drone_id):
        """Initialize a single drone's subscribers and services with position publisher"""
        try:
            # Create and store subscribers
            callback = self.create_callback(drone_id)
            callback2 = self.create_pos_callback(drone_id)
            
            state_sub = rospy.Subscriber(f'/uav{drone_id}/mavros/state', State, callback, queue_size=20)
            pos_sub = rospy.Subscriber(f"/uav{drone_id}/position_input", Pose, callback2, queue_size=20)
            
            # Store subscribers
            if len(self.state_subscribers) <= drone_id:
                self.state_subscribers.extend([None] * (drone_id - len(self.state_subscribers) + 1))
            if len(self.position_subscribers) <= drone_id:
                self.position_subscribers.extend([None] * (drone_id - len(self.position_subscribers) + 1))
                
            self.state_subscribers[drone_id] = state_sub
            self.position_subscribers[drone_id] = pos_sub
            
            # Initialize publisher
            if len(self.pos_publishers) <= drone_id:
                self.pos_publishers.extend([None] * (drone_id - len(self.pos_publishers) + 1))
            
            self.pos_publishers[drone_id] = rospy.Publisher(
                f"/uav{drone_id}/mavros/setpoint_position/local",
                PoseStamped,
                queue_size=20
            )
            
            # Initialize service clients
            rospy.wait_for_service(f"/uav{drone_id}/mavros/cmd/arming")
            rospy.wait_for_service(f"/uav{drone_id}/mavros/set_mode")
            
            if len(self.arming_clients) <= drone_id:
                self.arming_clients.extend([None] * (drone_id - len(self.arming_clients) + 1))
            if len(self.mode_clients) <= drone_id:
                self.mode_clients.extend([None] * (drone_id - len(self.mode_clients) + 1))
            
            self.arming_clients[drone_id] = rospy.ServiceProxy(f"/uav{drone_id}/mavros/cmd/arming", CommandBool)
            self.mode_clients[drone_id] = rospy.ServiceProxy(f"/uav{drone_id}/mavros/set_mode", SetMode)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error initializing UAV{drone_id}: {str(e)}")
            return False

    def initialize(self):
        """Initialize swarm control with proper position initialization"""
        rospy.loginfo("Initializing swarm control...")
        success = True
        
        # Initialize position arrays first
        self.curr_positions = [Point() for _ in range(self.no_of_drones)]
        self.curr_positions2 = [Point() for _ in range(self.no_of_drones)]
        
        # Set initial positions
        for i in range(self.no_of_drones):
            self.curr_positions[i].x = 0.0  # Spread drones along x-axis
            self.curr_positions[i].y = 0.0
            self.curr_positions[i].z = 2.0  # Initial height
            
            # Copy to curr_positions2
            self.curr_positions2[i] = Point()
            self.curr_positions2[i].x = self.curr_positions[i].x
            self.curr_positions2[i].y = self.curr_positions[i].y
            self.curr_positions2[i].z = self.curr_positions[i].z
        
        # Initialize subscribers and services
        for i in range(self.no_of_drones):
            if not self.initialize_single_drone(i):
                rospy.logerr(f"Failed to initialize UAV{i}")
                success = False
        
        if success:
            rospy.loginfo("Initialization complete")
        return success

    def wait_for_connections(self, timeout=60):
        # """Wait for all drones to connect with timeout"""
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            all_connected = True
            for i in range(self.no_of_drones):
                if not self.verify_drone_state(i):
                    all_connected = False
                    continue
                
                if not self.states[i].connected:
                    all_connected = False
                    rospy.logwarn_throttle(10, f"Waiting for UAV{i} to connect...")
            
            if all_connected:
                rospy.loginfo("All drones connected!")
                return True
                
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("Timeout waiting for drone connections")
                return False
                
            self.rate.sleep()

    def check_system_health(self, drone_id):
        """Check and fix common system health issues"""
        try:
            # Check if EKF2_AID_MASK parameter exists and set it if necessary
            try:
                param_get_result = self.param_get_clients[drone_id]("EKF2_AID_MASK")
                if param_get_result.success:
                    current_mask = param_get_result.value.integer
                    # Set vision position fusion and GPS fusion
                    desired_mask = 1 | (1 << 3)  # 1 for GPS, 1<<3 for vision position
                    if current_mask != desired_mask:
                        rospy.loginfo(f"Setting EKF2_AID_MASK for UAV{drone_id}")
                        self.param_set_clients[drone_id]("EKF2_AID_MASK", desired_mask)
            except Exception as e:
                rospy.logwarn(f"Could not set EKF2_AID_MASK for UAV{drone_id}: {str(e)}")

            # Set COM_RCL_EXCEPT parameter to allow arming without RC
            try:
                param_get_result = self.param_get_clients[drone_id]("COM_RCL_EXCEPT")
                if param_get_result.success:
                    current_value = param_get_result.value.integer
                    desired_value = 4  # Allow arming without RC
                    if current_value != desired_value:
                        rospy.loginfo(f"Setting COM_RCL_EXCEPT for UAV{drone_id}")
                        self.param_set_clients[drone_id]("COM_RCL_EXCEPT", desired_value)
            except Exception as e:
                rospy.logwarn(f"Could not set COM_RCL_EXCEPT for UAV{drone_id}: {str(e)}")

            return True
        
        except Exception as e:
            rospy.logerr(f"Error checking system health for UAV{drone_id}: {str(e)}")
            return False
        
    def arm_and_set_mode(self):
        """Arm all drones and set to OFFBOARD mode with system health checks"""
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        
        timeout = rospy.Time.now() + rospy.Duration(100)
        
        # Start setpoint publishing thread
        setpoint_thread = threading.Thread(target=self.send_setpoints_thread)
        setpoint_thread.daemon = True
        setpoint_thread.start()
        
        # Wait for setpoints to be published
        rospy.sleep(2.0)
        
        while not rospy.is_shutdown():
            all_ready = True
            
            for i in range(self.no_of_drones):
                if self.states[i] is None:
                    all_ready = False
                    continue

                # Check system health before attempting mode change or arming
                if not self.check_system_health(i):
                    all_ready = False
                    continue

                # Set to OFFBOARD mode if not already
                if self.states[i].mode != "OFFBOARD":
                    all_ready = False
                    try:
                        if self.mode_clients[i].call(offb_set_mode).mode_sent:
                            rospy.loginfo(f"OFFBOARD enabled for UAV{i}")
                            rospy.sleep(1.0)
                    except Exception as e:
                        rospy.logerr(f"Mode change failed for UAV{i}: {str(e)}")
                    continue

                # Attempt arming if in OFFBOARD and not armed
                if not self.states[i].armed:
                    all_ready = False
                    try:
                        if self.arming_clients[i].call(arm_cmd).success:
                            rospy.loginfo(f"UAV{i} armed successfully")
                            rospy.sleep(0.5)
                        else:
                            rospy.logwarn(f"Arming failed for UAV{i}")
                    except Exception as e:
                        rospy.logerr(f"Arming failed for UAV{i}: {str(e)}")
            
            if all_ready:
                rospy.loginfo("All drones armed and in OFFBOARD mode!")
                return True
                
            if rospy.Time.now() > timeout:
                rospy.logerr("Timeout while arming drones")
                return False
                
            self.rate.sleep()

    def send_setpoints_thread(self):
        """Continuous setpoint publishing thread with proper error handling"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        
        # Initialize pose orientation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        while not rospy.is_shutdown():
            try:
                for i in range(self.no_of_drones):
                    if i >= len(self.pos_publishers) or self.pos_publishers[i] is None:
                        continue
                        
                    if i >= len(self.curr_positions) or self.curr_positions[i] is None:
                        continue
                        
                    # Update and publish position
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position = self.curr_positions[i]
                    print(f"Taking UAV{i} to position {self.curr_positions[i].x,self.curr_positions[i].y,self.curr_positions[i].z}")
                    
                    try:
                        self.pos_publishers[i].publish(pose)
                        self.rate.sleep()
                    except Exception as e:
                        rospy.logerr(f"Error publishing setpoint for UAV{i}: {str(e)}")
                    
            except Exception as e:
                rospy.logerr(f"Error in setpoint thread: {str(e)}")
                
            self.rate.sleep()

    def main(self):
        if not self.initialize():
            rospy.logerr("Initialization failed")
            return

        if not self.wait_for_connections():
            rospy.logerr("Connection timeout")
            return

        if not self.arm_and_set_mode():
            rospy.logerr("Arming/mode setting failed")
            return

        try:
            while not rospy.is_shutdown():
                self.arm_msg_pub.publish("Armed")
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("User interrupted")

if __name__ == "__main__":
    try:
        control = swarm_control()
        control.main()
    except KeyboardInterrupt:
        rospy.signal_shutdown("User interrupted")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {str(e)}")
        rospy.signal_shutdown("Unexpected error occurred")