#! /usr/bin/env python3

import rospy
from sys import argv
from geometry_msgs.msg import Pose,PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from threading import Thread

current_state = State()
pose = PoseStamped()
publishing = False
stop_publishing = False

def state_cb(msg):
    global current_state
    current_state = msg

def pos_callback(msg):
    global curr_position
    curr_position = PoseStamped()
    curr_position.pose.position = msg.position
    
def setpoint_thread():
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 4

    # Send a few setpoints before starting
    while not rospy.is_shutdown():
        global publishing
        publishing = True
        # print(publishing)
        if stop_publishing:
            break
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    id = argv[1]
    print(f"UAV{id} initiated")
    rospy.init_node("offb_node_py")
    try:
        state_sub = rospy.Subscriber(f"/uav{id}/mavros/state", State,state_cb)

        local_pos_pub = rospy.Publisher(f"/uav{id}/mavros/setpoint_position/local", PoseStamped, queue_size=30)

        rospy.wait_for_service(f"/uav{id}/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy(f"/uav{id}/mavros/cmd/arming", CommandBool)

        rospy.wait_for_service(f"/uav{id}/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy(f"/uav{id}/mavros/set_mode", SetMode)

        arm_msg_pub = rospy.Publisher("/uavs/arm_msg",String,queue_size=10)

        curr_pos_sub = rospy.Subscriber(f"/uav{id}/position_input",Pose,pos_callback)

        rospy.set_param(f"/uav{id}/mavros/conn/timeout",30.0)
        rospy.set_param(f"/uav{id}/mavros/conn/heartbeat_rate",0.0)

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not current_state.connected):
            print(f"Drone {id} not connected yet")
            rate.sleep()

        # Send a few setpoints before starting
        setpoint_pub = Thread(target=setpoint_thread,daemon=True)
        setpoint_pub.start()
        rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        # last_req = rospy.Time.now()
        curr_position = pose

        while current_state.mode != "OFFBOARD":
            if publishing:
                if set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo(f"OFFBOARD enabled {id}")
                rospy.sleep(2)
            else:
                print("Waiting for Publisher" + str(publishing))
                rate.sleep()

        # Once in OFFBOARD, attempt arming
        while not current_state.armed:
            if publishing:
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo(f"Vehicle armed {id}")
                rospy.sleep(2)
            else:
                rate.sleep()
                    # last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            arm_msg_pub.publish(f"Armed UAV{id}")
            # rospy.sleep(0.5)
            
            if curr_position.pose.position == pose.pose.position:
                rate.sleep()
            else:
                stop_publishing = True
                rate.sleep()
                local_pos_pub.publish(curr_position)

    except KeyboardInterrupt:
        rospy.signal_shutdown()

