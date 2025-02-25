import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

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
            rospy.loginfo("Service did not process request: "+str(exc))
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
        vel = currentPose.twist.linear.x
        yaw = quaternion_to_euler(
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        )[2]

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 10

        straight_vel = 12
        turn_vel = 8

        turn_dist = 5

        waypoint_x = future_unreached_waypoints[0][0]
        waypoint_y = future_unreached_waypoints[0][1]

        distance_to_waypoint = math.sqrt((waypoint_x - curr_x)**2 + (waypoint_y - curr_y)**2)

        if distance_to_waypoint > turn_dist:
            target_velocity = straight_vel
        else:
            target_velocity = turn_vel

        ####################### TODO: Your TASK 2 code ends Here #######################

        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_steering = 0

        # lookahead distance `ld`
        ld = ((target_point[0] - curr_x)**2 + (target_point[1] - curr_y)**2)**0.5

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
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
