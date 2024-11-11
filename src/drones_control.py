#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose,PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

no_of_drones = 10

class swarm_control:
    def __init__(self):
        rospy.init_node("swarm_control",anonymous=True)
        self.rate = rospy.Rate(10)
        self.states = [None] * no_of_drones
        self.pos_publishers = [None] * no_of_drones
        self.arming_clients = []
        self.mode_clients = []
        self.curr_positions = [None] * no_of_drones
        self.arm_msg_pub = rospy.Publisher("/uavs/arm_msg",String,queue_size=10)
        
    def create_callback(self,drone_id):
        def callback(data):
            state_var = State()
            state_var.connected = data.connected
            state_var.armed = data.armed
            state_var.guided = data.guided
            state_var.mode = data.mode
            state_var.system_status = data.system_status
            self.states[drone_id] = state_var
        return callback

    def create_pos_callback(self,drone_id):
        def callback(data):
            pos = Pose()
            pos.pose.position = data.position
            self.curr_positions[drone_id] = pos.pose.position
        return callback

    def initialize(self,n):
        for i in range(n):
            callback = self.create_callback(i)
            callback2 = self.create_pos_callback(i)
            sub = rospy.Subscriber(f'/uav{i}/mavros/state',State,callback,queue_size=20)
            self.position_sub = rospy.Subscriber(f"/uav{i}/position_input",Pose,callback2)
            local_pos_pub = rospy.Publisher(f"/uav{i}/mavros/setpoint_position/local", PoseStamped, queue_size=20)

            rospy.wait_for_service(f"/uav{i}/mavros/cmd/arming")
            arming_client = rospy.ServiceProxy(f"/uav{i}/mavros/cmd/arming", CommandBool)

            rospy.wait_for_service(f"/uav{i}/mavros/set_mode")
            set_mode_client = rospy.ServiceProxy(f"/uav{i}/mavros/set_mode", SetMode)

            self.pos_publishers[i] = local_pos_pub
            self.arming_clients.append(arming_client)
            self.mode_clients.append(set_mode_client)
        return True

    def main(self):
        while not self.initialize(no_of_drones):
            self.rate.sleep()
        while not rospy.is_shutdown():
            all_connected = True
            for state in self.states:
                if state is None or not state.connected:
                    all_connected = False
                    break
            if all_connected:
                print("All connected")
                break
            self.rate.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        # Send a few setpoints before starting
        i = 0
        for j in range(100):
            if(rospy.is_shutdown()):
                break

            if i > 9:
                i = 0
            self.pos_publishers[i].publish(pose)
            i += 1
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        # last_req = rospy.Time.now()
        try:
            while self.states[no_of_drones-1].armed != True:
                for i in range(no_of_drones):
                    if self.states[i] is None:
                        continue
                    if self.states[i].mode != "OFFBOARD":
                        if(self.mode_clients[i].call(offb_set_mode).mode_sent == True):
                            rospy.loginfo(f"OFFBOARD enabled for uav{i}")
                            rospy.sleep(1.0)
                        # last_req = rospy.Time.now()
                        continue
                    if not self.states[i].armed:
                        if(self.arming_clients[i].call(arm_cmd).success == True):
                            rospy.loginfo(f"Vehicle uav{i} armed")
                            rospy.sleep(0.5)
                        # last_req = rospy.Time.now()
        except KeyboardInterrupt:
            rospy.signal_shutdown()

        self.arm_msg_pub.publish("Armed")    

        while any(pos is None for pos in self.curr_positions):
            print("Give curr pos")
            self.rate.sleep()
        print(self.curr_positions)
        while(not rospy.is_shutdown()):
            for i in range(no_of_drones):
                if self.curr_positions[i] == None:
                    print("wtf")
                    continue
                self.pos_publishers[i].publish(self.curr_positions[i])
                self.rate.sleep()
            print(self.curr_positions)

if __name__ == "__main__":
    try:
        control = swarm_control()
        control.main()
    except KeyboardInterrupt:
        rospy.signal_shutdown()