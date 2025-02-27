import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time
import matplotlib.pyplot as plt
import numpy as np

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=1)
        self.prev_vel = 0
        self.L = 1.75  # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        self.accelerations =[]
        self.times = []
        self.time = 0

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose: ModelState):

        ####################### TODO: Your TASK 1 code starts Here #######################

        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        vel = (currentPose.twist.linear.x ** 2 + currentPose.twist.linear.y ** 2) ** 0.5
        yaw = quaternion_to_euler(
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        )[2]

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw  # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################

        if len(future_unreached_waypoints) > 2:
            first_point = future_unreached_waypoints[0]
            second_point = future_unreached_waypoints[1]
        else:
            first_point = future_unreached_waypoints[0]
            second_point = first_point

        ang1 = math.atan2(first_point[1] - curr_y, first_point[0] - curr_x)
        ang2 = math.atan2(second_point[1] - curr_y, second_point[0] - curr_x)
        diff = ang1 - ang2
        while (diff > math.pi):
            diff = diff - 2 * math.pi
        while (diff < -math.pi):
            diff = diff + 2 * math.pi
        diff = np.abs(diff)

        if (diff > 0.25):
            target_velocity = 5
        elif (diff > 0.15):
            target_velocity = 14
        elif (diff > 0.1):
            target_velocity = 15
        elif (diff > 0.04):
            target_velocity = 16
        elif (diff > 0.03):
            target_velocity = 18
        else:
            target_velocity = 20

        print(target_velocity, '\t', curr_vel)

        ####################### TODO: Your TASK 2 code ends Here #######################

        return target_velocity

    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        PROX_THRESH = 3.5

        ####################### TODO: Your TASK 3 code starts Here #######################

        if len(future_unreached_waypoints) > 3:
            target_point_2 = future_unreached_waypoints[1]

            # lookahead distance `ld`
            ld = ((target_point[0] - curr_x) ** 2 + (target_point[1] - curr_y) ** 2) ** 0.5
            ld_2 = ((target_point_2[0] - curr_x) ** 2 + (target_point_2[1] - curr_y) ** 2) ** 0.5

            # print(f"Distance til next 2 targets: {ld}, {ld_2}")

            # angle between
            alpha = math.atan2(target_point[1] - curr_y, target_point[0] - curr_x) - curr_yaw
            alpha_2 = math.atan2(target_point_2[1] - curr_y, target_point_2[0] - curr_x) - curr_yaw

            # steering angle
            target_steering_1 = math.atan2(2 * self.L * math.sin(alpha), ld)
            target_steering_2 = math.atan2(2 * self.L * math.sin(alpha_2), ld_2)

            weight = 0 if ld < PROX_THRESH else 1 - 1 / (ld - PROX_THRESH + 1)
            target_steering = weight * target_steering_1 + (1 - weight) * target_steering_2

        else:
            # lookahead distance `ld`
            ld = ((target_point[0] - curr_x) ** 2 + (target_point[1] - curr_y) ** 2) ** 0.5

            # angle between
            alpha = math.atan2(target_point[1] - curr_y, target_point[0] - curr_x) - curr_yaw

            # steering angle
            target_steering = math.atan2(2 * self.L * math.sin(alpha), ld)

        ####################### TODO: Your TASK 3 code ends Here #######################

        return target_steering

    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel - self.prev_vel) * 100  # Since we are running in 100Hz
            self.accelerations.append(acceleration)
            self.time += 0.01
            self.times.append(self.time)

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point,
                                                               future_unreached_waypoints)

        # Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def plot_acceleration(self):
        plt.plot(self.times, self.accelerations)
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s^2)')
        plt.title('Acceleration Profile')
        plt.show()

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
