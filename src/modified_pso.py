import rospy
import numpy as np
import pandas as pd
from matplotlib import pyplot
import math as m
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import sympy as sp

# Set the random seed for reproducibility
np.random.seed(42)
random.seed(42)  # Set the seed for the built-in random module

class params:
    # Constriction coefficients
    kappa = 1
    phi1 = 2.05
    phi2 = 2.05
    phi = phi1 + phi2
    chi = 1.37              # 2*kappa/abs(2-phi-m.sqrt(phi**2-4*phi))

    # other params
    max_iter = 100        # maximum number of iterations
    nPart = 3              # no of drones/particles
    w = chi                 # inertia coefficient
    wdamp = 0.7             # damping coefficent for inertia
    c1 = chi * phi1         # personal acceleration coefficient
    c2 = chi * phi2         # social acceleration coefficient
    showIterinfo = True 
    a=0.2
    b=0.35   # Flag for showing iteration information

    pose = Pose()
    

class particle(params):
    position = np.zeros(shape=(params.nPart, 3), dtype=np.float64)                     
    velocity = np.zeros(shape=(params.nPart, 3), dtype=np.float64)                   
    cost = np.zeros(shape=(params.nPart, 1), dtype=np.float64)                  
    Best_cost = np.zeros(shape=(params.nPart, 1), dtype=np.float64)          
    Best_position = np.zeros(shape=(params.nPart, 3), dtype=np.float64)          
    Global_best_cost = np.array([1000])
    Global_best_position = np.zeros(shape=(1, 3), dtype=np.float64)
    vmax = np.array([1.2, 1.2, 1.2])
    vmin = np.array([-1.2, -1.2, -1.2])

    waypoints = pd.DataFrame(position, columns=['x', 'y', 'z'])

class PSO(particle):
    def __init__(self):
        # rospy.init_node("pso_search", anonymous=True)
        # for i in range(params.nPart):
        #     callback = self.create_callback(i)
        #     local_pos_sub = rospy.Subscriber(f"/uav{i}/mavros/local_position/odom", Odometry, callback)
        #     pos_pub = rospy.Publisher(f"/uav{i}/position_input", Pose, queue_size=10)
        # self.Rate = rospy.rate(10)
        self.nvar = 3
        self.varMin = 0
        self.varMax = 10
        self.x=3
        self.y=3
        self.z=3       
        self.goal = np.array([self.x, self.y, self.z])
        for i in range(params.nPart):
            self.positions = np.array([np.linspace(particle.position[i,0],self.x,50)],[np.linspace(particle.position[i,1],self.y,50)],[np.linspace(particle.position[i,2],self.z,50)])

    # Cost function to minimize the distance error 
    # between the goal and current position of each drone
    def costfn(self, position):
        cost = 0
        for i in range(len(position)):
            cost += np.subtract(self.goal[i], position[i])**2
        return m.sqrt(cost)
    
    def initialization(self):
        for i in range(params.nPart):
            # Generate random positions for particles
            particle.position[i, :] = np.array([
                random.randint(self.varMin, self.varMax),
                random.randint(self.varMin, self.varMax),
                random.randint(self.varMin, self.varMax)
            ])
            particle.cost[i] = self.costfn(particle.position[i, :])
            particle.Best_cost[i] = particle.cost[i]
            particle.Best_position[i, :] = particle.position[i, :]

            if particle.cost[i] < particle.Global_best_cost:
                particle.Global_best_cost = particle.cost[i]
                particle.Global_best_position[0, :] = particle.position[i, :]
        print(particle.position)
        self.Bestcosts = np.zeros(shape=(params.max_iter, 1))

    
    def main (self):
        self.initialization()
        learning_rate = 2
        for j in range(params.max_iter):
            for i in range(params.nPart):
                # Update the velocity and cost
                # particle.velocity[i, :] = params.w * particle.velocity[i, :] + \
                #     params.c1 * random.uniform(0.3, 0.35) * np.subtract(particle.Best_position[i, :], particle.position[i, :]) + \
                #     params.c2 * random.uniform(0.3, 0.35) * np.subtract(particle.Global_best_position[0, :], particle.position[i, :])
                
                particle.cost[i] = self.costfn(self.find_derivative_and_solutions())
                particle.position[i,:] += self.positions[i,:]

                if particle.cost[i] < particle.Best_cost[i]:
                    particle.Best_cost[i] = particle.cost[i]
                    particle.Best_position[i, :] = particle.position[i, :]

                    if particle.Best_cost[i] < particle.Global_best_cost:
                        particle.Global_best_cost = particle.Best_cost[i]
                        particle.Global_best_position[0, :] = particle.position[i, :]
            
            self.Bestcosts[j] = particle.Global_best_cost

            if params.showIterinfo:
                print("Iteration", str(j), ": Best Cost", str(self.Bestcosts[j]))

            params.w *= params.wdamp
        print(particle.position)

    # def compute_gradient(self, position):
    #     # Compute the gradient of the cost function with respect to the position
    #     gradient = np.zeros(3)
    #     for k in range(3):
    #         epsilon = 1e-6
    #         position_plus_epsilon = position.copy()
    #         position_plus_epsilon[k] += epsilon
    #         position_minus_epsilon = position.copy()
    #         position_minus_epsilon[k] -= epsilon
            
    #         gradient[k] = (self.costfn(position_plus_epsilon) - self.costfn(position_minus_epsilon)) / (2 * epsilon)
        
    #     return gradient

    def find_derivative_and_solutions(self):
        """
        This function takes coefficients of a quadratic equation (ax^2 + bx + c),
        differentiates it, and finds the values of x where the derivative equals zero.

        Parameters:
        a (float): Coefficient of x^2
        b (float): Coefficient of x
        c (float): Constant term

        Returns:
        tuple: Original equation, derivative, and solutions for x
        """
        # Step 1: Define the variable
        
        x = sp.symbols('x')
        derivatives = []
        equations = []
        dev_solutions = []
        for i in range(3):
            a=1
            b=-2*self.goal[i]
            c=self.goal[i]**2
            # Step 2: Define the quadratic equation
            quadratic_equation = a * x**2 + b * x + c

            # Step 3: Differentiate the equation with respect to x
            derivative = sp.diff(quadratic_equation, x)
            derivatives.append(derivative)
            # Step 4: Set the derivative equal to zero
            equation = sp.Eq(derivative, 0)
            equations.append(equation)
            # Step 5: Solve for x
            solutions = sp.solve(equation, x)
            dev_solutions.append(solutions)
            # Step 6: Print the results
            # print(f"Original Equation: {original_eq}")
            # print(f"Derivative: {derivative}")
            # print(f"Solutions for x (where derivative = 0): {solutions}")

        return dev_solutions

    
    
if __name__ == "__main__":
    optimize = PSO()
    optimize.main()