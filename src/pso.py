#!/usr/bin/env python3

import rospy
import numpy as np
import math as m
import random
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from params import params
from threading import Thread

class particle(params):
    position = np.zeros(shape=(params.nPart,3),dtype=np.float64)
    velocity = np.zeros(shape=(params.nPart,3),dtype=np.float64)                   
    cost = np.zeros(shape=(params.nPart,1),dtype=np.float64)                  
    Best_cost = np.zeros(shape=(params.nPart,1),dtype=np.float64)          
    Best_position = np.zeros(shape=(params.nPart,3),dtype=np.float64)          
    Global_best_cost = np.array([1000])
    Global_best_position = np.zeros(shape=(1,3),dtype=np.float64)
    vmax = np.array([2.0,2.0,2.0])
    vmin = np.array([-2.0,-2.0,2.0])

class PSO(particle):
    def __init__(self):
        rospy.init_node("pso_search")    
        self.Rate = rospy.Rate(20)
        self.nvar = 3
        self.varMin = 0
        self.varMax = 10
        self.goal = np.array([5.0,5.0,5.0])
        self.pose = Pose()
        arm_msg = String()
        arm_msg = "None"
        self.arm_msg = [arm_msg] * params.nPart
        self.initial_positions = [None] * params.nPart
        self.stop_update = False
        self.pos_pubs = []
        self.arm_msg_sub = rospy.Subscriber("/uavs/arm_msg",String,self.cback)
        self.queuing_lat = rospy.Publisher("/uavs/queuing",String,queue_size=20)
        for i in range(params.nPart):
            local_pos_sub = rospy.Subscriber(f"/gazebo/model_states",ModelStates,self.callback)
            pos_pub = rospy.Publisher(f"/uav{i}/position_input",Pose,queue_size=20)
            self.pos_pubs.append(pos_pub)

    def cback(self,data):
        # print(int(data.data[9]))
        self.arm_msg[int(data.data[9])] = data.data 
        # print(self.arm_msg)

    def callback(self,data):
        pose = Pose()
        if self.stop_update != True:
            for i in range(len(data.name)):
                if data.name[i][0:4] == "iris":
                    uav = int(data.name[i][4])
                    pose.position = data.pose[i].position
                    self.initial_positions[uav] = np.array([round(pose.position.x,0),round(pose.position.y,0),round(pose.position.z,0)])
                    self.stop_update = True

    # Cost function to minimize the distance error 
    # between the goal and current position of each drone
    def costfn(self,position):
        cost = 0
        for i in range(len(position)):
            cost += np.subtract(self.goal[i],position[i])**2

        return m.sqrt(cost)
    
    def queuing_test_thread(self):
        i = 0
        while not rospy.is_shutdown():
            if i > 5:
                break
            msg = String()
            curr_time = str(rospy.get_rostime().nsecs)
            msg.data = f"Msg {i} sent at " + curr_time
            print(i,msg.data)
            self.queuing_lat.publish(msg)
            self.Rate.sleep()
            i += 1
    
    def initialization(self):
        while (not rospy.is_shutdown() and any(pos is None for pos in self.initial_positions)):
            self.Rate.sleep()
        for i in range(params.nPart):
            particle.position[i,:] = self.initial_positions[i]
            particle.cost[i] = self.costfn(particle.position[i,:])
            particle.Best_cost[i] = particle.cost[i]
            particle.Best_position[i,:] = particle.position[i,:]

            if particle.cost[i] < particle.Global_best_cost:
                particle.Global_best_cost = particle.cost[i]
                particle.Global_best_position[0,:] = particle.position[i,:]
        print(particle.position)
        self.Bestcosts = np.zeros(shape=(params.max_iter,1),dtype=np.float64)

    def main(self):
        queuing_thread = Thread(target=self.queuing_test_thread,daemon=True)
        queuing_thread.start()
        self.initialization()
        while (not rospy.is_shutdown() and any(x == "None" for x in self.arm_msg)):
            print(self.arm_msg)
            self.Rate.sleep()
        self.Rate.sleep()
        for j in range(params.max_iter):
            for i in range(params.nPart):
                particle.velocity[i,:] = params.w * particle.velocity[i,:] + params.c1 * random.uniform(0.3,0.35) * np.subtract(particle.Best_position[i,:],particle.position[i,:]) + params.c2 * random.uniform(0.3,0.35) * np.subtract(particle.Global_best_position[0,:],particle.position[i,:])

                if np.all(particle.velocity[i,:] > particle.vmax):
                    particle.velocity[i,:] = particle.vmax
                elif np.all(particle.velocity[i,:] < particle.vmin):
                    particle.velocity[i,:] = particle.vmin
                
                particle.position[i,:] += particle.velocity[i,:]
                self.pose.position.x = particle.position[i, 0]
                self.pose.position.y = particle.position[i, 1]
                self.pose.position.z = particle.position[i, 2]
                self.pose.orientation.x = 0
                self.pose.orientation.y = 0
                self.pose.orientation.z = 0
                self.pose.orientation.w = 1

                print(f"j = {j} and i = {i}")
                self.pos_pubs[i].publish(self.pose)

                particle.cost[i] = self.costfn(particle.position[i,:])

                if particle.cost[i] < particle.Best_cost[i]:
                    particle.Best_cost[i] = particle.cost[i]
                    particle.Best_position[i,:] = particle.position[i,:]

                    if particle.Best_cost[i] < particle.Global_best_cost:
                        particle.Global_best_cost = particle.Best_cost[i]
                        particle.Global_best_position[0,:] = particle.position[i,:]
            self.Rate.sleep()
            self.Bestcosts[j] = particle.Global_best_cost

            if params.showIterinfo:
                print("Iteration",str(j),": Best Cost",str(self.Bestcosts[j]))

            params.w *= params.wdamp
        print(particle.position)
        try:
            while not rospy.is_shutdown():
                self.Rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown()

if __name__ == "__main__":
    optimize = PSO()
    optimize.main()