import numpy as np
import matplotlib.pyplot as plt
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

import random

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta) * t 
    dy = vr * np.sin(curr_theta) * t
    dtheta = delta * t
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob: Robot, world: Maze, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # # (Default) The whole map
            # x = np.random.uniform(0, world.width)
            # y = np.random.uniform(0, world.height)


            ## first quadrant
            # x = 
            # y =
            x = np.random.uniform(world.width/2, world.width)
            y = np.random.uniform(world.height/2, world.height)

            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            return 1./len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))


    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####
        weights = []
        for p in self.particles:
            measurements_particle = p.read_sensor() 
            # Assign weights using Gaussian
            w = self.weight_gaussian_kernel(readings_robot, measurements_particle, std=500)
            weights.append(w)

        total = sum(weights)
        weights = np.array(weights)
        weights = weights / total

        for i, p in enumerate(self.particles):
            p.weight = weights[i]

        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()
        ## TODO #####

        weights = [p.weight for p in self.particles]
        
        cumulative_sum = np.cumsum(weights)
        cumulative_sum[-1] = 1.0

        for i in range(self.num_particles):
            rand_val = random.uniform(0, cumulative_sum[-1])
            idx = np.searchsorted(cumulative_sum, rand_val)
            chosen_particle = self.particles[idx]

            new_particle = Particle(
                x=chosen_particle.x,
                y=chosen_particle.y,
                heading=chosen_particle.heading,
                maze=self.world,
                sensor_limit=self.sensor_limit,
                noisy=True
            )
            particles_new.append(new_particle)

        ###############

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        dt = 0.01
        for (vr, delta) in self.control:
            for p in self.particles:
                dx, dy, dheading = vehicle_dynamics(dt, [p.x, p.y, p.heading], vr, delta)
                p.x += dx
                p.y += dy
                p.heading += dheading

        ###############
        self.control = []


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        timestep = []
        error_pos = []
        error_head = []
        fig, ax = plt.subplots(2,1)
        ax1 = ax[0]
        ax2 = ax[1]
        pos_err_line, = ax1.plot([], [], 'b',)
        head_err_line, = ax2.plot([], [], 'r')
        ax1.set_title("Position Error")
        ax2.set_title("Heading Error")
        fig.show()
        while True:
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####
            count = count + 1
            timestep.append(count)
            # 1
            self.particleMotionModel()
            reading = self.bob.read_sensor()
            self.updateWeight(reading)
            self.resampleParticle()

            # 2
            self.world.clear_objects()
            self.world.show_particles(self.particles)
            self.world.show_robot(self.bob)
            self.world.show_estimated_location(self.particles)

            # 3
            x_error = self.bob.x - self.particles[0].x
            y_error = self.bob.y - self.particles[0].y
            heading_error = self.bob.heading - self.particles[0].heading
            error_pos.append(np.linalg.norm([x_error, y_error]))
            error_head.append(heading_error)

            pos_err_line.set_xdata(timestep)
            pos_err_line.set_ydata(error_pos)
            head_err_line.set_xdata(timestep)
            head_err_line.set_ydata(error_head)

            ax1.relim()
            ax2.relim()
            ax1.autoscale_view()
            ax2.autoscale_view()

            fig.canvas.draw()
            fig.canvas.flush_events()

            ###############
