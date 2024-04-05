import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

import math
import numpy as np


class VehicleUpdate(Node):

    """
    This code is used to simulate the vehicle state update. 
    It uses the Ackermann model to calculate the new vehicle state base on the past state and the new velocity and steering angle.
    It has been suppoed :
                            1. that the vehicle is a bicycle model
                            2. actual vehicle speed and steering angle are obtained from the CAN bus

    How to calculate the new vehicle state: 
    Acculate the vehicle position and orientation based on the current velocity and steering angle.
    
    """

    def __init__(self):
        super().__init__('vehicle_update_node')

        # Vehicle parameters
        self.L = 2.9                # wheelbase [m]
        self.r_wheels = 0.34        # tire radius [m]
        
        self.vehicle_x = 0.0        # vehicle x position [m]
        self.vehicle_y = 0.0        # vehicle y position [m]
        self.vehicle_yaw = 0.0      # vehicle yaw angle [rad]
        self.dt = 0.1               # time step [s]

        self.new_velocity = 0.0     # actual vehicle velocity [m/s]
        self.new_steering = 0.0     # actual vehicle steering angle [rad]

        # switch test mode between fake data and real data
        self.use_simulated_data = True

        if self.use_simulated_data:
            # fake data
            self.dd_vel_subscription = self.create_subscription(Twist, 'driving_decision_vel', self.vehicle_vel_callback, 1)
            self.dd_steering_subscription = self.create_subscription(Float32, 'driving_decision_steering_ang', self.vehicle_steering_callback, 1)
        else:
            # real data
            self.sub_can_id451_ = self.create_subscription(DecodedCanMsg, "/can/from_can_bus/msgid451", self.can_id451_callback, 1)
            self.sub_can_id1281_ = self.create_subscription(DecodedCanMsg, "/can/from_can_bus/msgid1281", self.can_id1281_callback, 1)

        self.vehicle_pose_publisher = self.create_publisher(Pose2D, 'vehicle_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_vehicle_state)

    def vehicle_vel_callback(self, msg):

        self.new_velocity = msg.linear.x
        #self.get_logger().info('I heard: vehicle velocity is =%f' % (self.new_velocity))
    
    def vehicle_steering_callback(self, msg):

        self.new_steering = msg.data
        #self.get_logger().info('I heard: vehicle steering is =%f' % (self.new_steering))  

    
    def can_id451_callback(self, msg_id451):
        # CAN data callbacks
        whl_spd_fl_ = msg_id451.sgnl_val[0]
        whl_spd_fr_ = msg_id451.sgnl_val[1]
        whl_spd_rl_ = msg_id451.sgnl_val[2]
        whl_spd_rr_ = msg_id451.sgnl_val[3]
        avg_wheel_speed = ((whl_spd_fl_ + whl_spd_fr_ + whl_spd_rl_ + whl_spd_rr_) / 4)     # average rotation speed of the wheels in [RPM]
        v_fzg_ = (avg_wheel_speed * 2 * math.pi * self.r_wheels_ * 60) / 1000               # average vehicle speed in [km/h]
        self.new_velocity = v_fzg_ / 3.6                                                    # real vehicle speed in [m/s]

    def can_id1281_callback (self, msg_id1281): 
        
        current_st_agl_front_ = msg_id1281.sgnl_val[0] / self.magic_navya_steering_factor_  # front steering angle in [°]
        current_st_agl_rear_ = msg_id1281.sgnl_val[1] / self.magic_navya_steering_factor_   # rear steering angle in [°]
        
        self.new_steering = current_st_agl_front_  * math.pi / 180                          # real front steering angle in [rad]


    def update_vehicle_state(self):
        """
        update the vehicle's new position and orientation base on past pose and new velocity and steering angle
        
        """                                             
        
        self.vehicle_x += self.new_velocity * math.cos(self.vehicle_yaw) * self.dt
        self.vehicle_y += self.new_velocity * math.sin(self.vehicle_yaw) * self.dt
        self.vehicle_yaw += self.new_velocity / self.L * math.tan(self.new_steering) * self.dt
        
    
    def publish_vehicle_state(self):

        self.update_vehicle_state()
        
        vehicle_pose = Pose2D()
        vehicle_pose.x = self.vehicle_x
        vehicle_pose.y = self.vehicle_y
        vehicle_pose.theta = self.vehicle_yaw
        
        self.vehicle_pose_publisher.publish(vehicle_pose)

def main(args=None):
    rclpy.init(args=args)
    update_vehicle_state_node = VehicleUpdate()
    rclpy.spin(update_vehicle_state_node)
    update_vehicle_state_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()