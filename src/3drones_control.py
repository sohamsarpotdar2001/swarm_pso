#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose,PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

no_of_drones = 3
for i in range(no_of_drones):
    globals()[f"state_{i}"] = State()

class swarm_control:
    def __init__(self):
        rospy.init_node("swarm_control",anonymous=True)
        self.rate = rospy.Rate(30)
        self.states = [None] * no_of_drones
        self.pos_publishers = []
        self.arming_clients = []
        self.mode_clients = []
        # self.curr_positions = [None] * no_of_drones
        
    def create_callback(self,drone_id):
        def callback(data):
            state_var = globals()[f"state_{drone_id}"]
            state_var.connected = data.connected
            state_var.armed = data.armed
            state_var.guided = data.guided
            state_var.mode = data.mode
            state_var.system_status = data.system_status
            self.states[drone_id] = state_var
        return callback

    # def create_pos_callback(self,drone_id):
    #     def callback(data):
    #         pos = globals()[f"pos{drone_id}"]
    #         pos.pose.position = data.position
    #         self.curr_positions[drone_id] = pos.pose.position
    #     return callback

    def initialize(self,n):
        for i in range(n):
            callback = self.create_callback(i)
            # callback2 = self.create_pos_callback(i)
            sub = rospy.Subscriber(f'/uav{i}/mavros/state',State,callback,queue_size=20)
            # position_sub = rospy.Subscriber(f"/uav{i}/position_input",Pose,callback2)
            local_pos_pub = rospy.Publisher(f"/uav{i}/mavros/setpoint_position/local", PoseStamped, queue_size=20)

            rospy.wait_for_service(f"/uav{i}/mavros/cmd/arming")
            arming_client = rospy.ServiceProxy(f"/uav{i}/mavros/cmd/arming", CommandBool)

            rospy.wait_for_service(f"/uav{i}/mavros/set_mode")
            set_mode_client = rospy.ServiceProxy(f"/uav{i}/mavros/set_mode", SetMode)

            self.pos_publishers.append(local_pos_pub)
            self.arming_clients.append(arming_client)
            self.mode_clients.append(set_mode_client)

    def main(self):
        self.initialize(no_of_drones)
        while not rospy.is_shutdown():
            all_connected = True
            for state in self.states:
                if state is None or not state.connected:
                    all_connected = False
                    break
            if all_connected:
                break
            self.rate.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):  
            for i in range(no_of_drones):
                if self.states[i] is None:
                    print(0)
                    continue
                if(self.states[i].mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.mode_clients[i].call(offb_set_mode).mode_sent == True):
                        rospy.loginfo(f"OFFBOARD enabled for uav{i}")
                    print(1)
                    print(self.states[i].mode)
                    last_req = rospy.Time.now()
                elif(not self.states[i].armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                        if(self.arming_clients[i].call(arm_cmd).success == True):
                            rospy.loginfo(f"Vehicle uav{i} armed")
                        print(2)
                        last_req = rospy.Time.now()
                print(self.states[i].armed)
                print(f"-------------------------------{i}")

            # for i in range(no_of_drones):
            #     if self.curr_positions[i] == None:
            #         print("wtf")
            #         continue
            #     print(3)
            #     self.pos_publishers[i].publish(self.curr_positions[i])
            # print(self.states)
            # print(self.curr_positions)
                self.pos_publishers[i].publish(pose)
                self.rate.sleep()

if __name__ == "__main__":
    try:
        control = swarm_control()
        control.main()
    except KeyboardInterrupt:
        rospy.signal_shutdown()