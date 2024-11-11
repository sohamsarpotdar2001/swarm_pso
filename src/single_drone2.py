#! /usr/bin/env python3

import rospy
from sys import argv
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from threading import Thread

current_state = State()
pose = PoseStamped()
publishing = False
stop_publishing = False
queuing = False

# Latency tracking variables
transmission_start_time = 0
processing_start_time = 0
last_message_time = 0
queue_latency = 0

def state_cb(msg):
    global current_state
    current_state = msg

def pos_callback(msg):
    global curr_position, processing_start_time
    # Mark processing start time
    processing_start_time = rospy.get_rostime().nsecs
    
    curr_position = PoseStamped()
    curr_position.pose.position = msg.position
    
    # Calculate processing latency
    rospy.sleep((0.3 * float(id)/10.0) + 0.3)
    processing_latency = rospy.get_rostime().nsecs - processing_start_time
    latency_pub.publish(Float64(processing_latency))
    
def queuing_callback(msg):
    global queue_latency, queuing
    if msg.data[4] == "5":
        queuing = True
        rospy.sleep((0.05 * float(id)/10.0) + 0.05)
        curr_time = rospy.get_rostime().nsecs
        prev_time = float(msg.data[14:])
        print("Current time = ",curr_time," and Previous time = ",prev_time)
        queue_latency = curr_time - prev_time

def calculate_transmission_latency():
    global transmission_start_time, last_message_time
    if last_message_time > 0:
        rospy.sleep((0.02465 * float(id)/10.0) + 0.02465)
        transmission_latency = rospy.get_rostime().nsecs - transmission_start_time
        trans_latency_pub.publish(Float64(transmission_latency))
    transmission_start_time = rospy.get_rostime().nsecs
    last_message_time = transmission_start_time
    
def setpoint_thread():
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 5

    while not rospy.is_shutdown():
        global publishing, queue_entry_time
        publishing = True
        if stop_publishing:
            break
        
        local_pos_pub.publish(pose)        
        rate.sleep()

if __name__ == "__main__":
    id = argv[1]
    print(f"UAV{id} initiated")
    rospy.init_node("offb_node_py2")
    
    try:
        # Original subscribers and publishers
        state_sub = rospy.Subscriber(f"/uav{id}/mavros/state", State, state_cb)
        local_pos_pub = rospy.Publisher(f"/uav{id}/mavros/setpoint_position/local", PoseStamped, queue_size=30)
        arm_msg_pub = rospy.Publisher("/uavs/arm_msg", String, queue_size=10)
        curr_pos_sub = rospy.Subscriber(f"/uav{id}/position_input", Pose, pos_callback)

        # New latency publishers
        latency_pub = rospy.Publisher(f"/uav{id}/processing_latency", Float64, queue_size=10)
        trans_latency_pub = rospy.Publisher(f"/uav{id}/transmission_latency", Float64, queue_size=10)
        queue_latency_sub = rospy.Subscriber("/uavs/queuing", String, queuing_callback)
        queue_latency_pub = rospy.Publisher("/uavs/queuing_latency",Float64,queue_size=20)

        rospy.wait_for_service(f"/uav{id}/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy(f"/uav{id}/mavros/cmd/arming", CommandBool)

        rospy.wait_for_service(f"/uav{id}/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy(f"/uav{id}/mavros/set_mode", SetMode)

        rospy.set_param(f"/uav{id}/mavros/conn/timeout", 30.0)
        rospy.set_param(f"/uav{id}/mavros/conn/heartbeat_rate", 0.0)

        rate = rospy.Rate(20)
        while not queuing:
            print("Not recieved Queue msg yet")
            rate.sleep()

        while(not rospy.is_shutdown() and not current_state.connected):
            print(f"Drone {id} not connected yet")
            rate.sleep()

        setpoint_pub = Thread(target=setpoint_thread, daemon=True)
        setpoint_pub.start()
        rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        curr_position = pose

        while current_state.mode != "OFFBOARD":
            if publishing:
                if set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo(f"OFFBOARD enabled {id}")
                rospy.sleep(2)
            else:
                print("Waiting for Publisher" + str(publishing))
                rate.sleep()

        while not current_state.armed:
            if publishing:
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo(f"Vehicle armed {id}")
                rospy.sleep(2)
            else:
                rate.sleep()
        rate.sleep()
        while not rospy.is_shutdown():
            arm_msg_pub.publish(f"Armed UAV{id}")
            queue_latency_pub.publish(Float64(queue_latency))
            calculate_transmission_latency()

            if curr_position.pose.position == pose.pose.position:
                rate.sleep()
            else:
                if curr_position.pose.position != pose.pose.position:
                    stop_publishing = True
                    rate.sleep()
                    local_pos_pub.publish(curr_position)
                    rospy.loginfo(f"Taking UAV{id} to {curr_position.pose.position}")

    except KeyboardInterrupt:
        rospy.signal_shutdown()