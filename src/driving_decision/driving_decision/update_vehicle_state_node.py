import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

import math
import numpy as np


class VehicleUpdate(Node):

    def __init__(self):
        super().__init__('vehicle_update_node')

        # Fahrzeug Parameter
      
        self.L = 2.9  # [m] Radstand
        self.r_wheels = 0.34  # Reifenradius in Metern
        

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0

        self.dt = 0.1 

        self.new_velocity = 0.0
        self.new_steering = 0.0
        self.vehicle_v = 0.0

        self.use_simulated_data = True

        if self.use_simulated_data:
            self.dd_vel_subscription = self.create_subscription(Twist, 'driving_decision_vel', self.vehicle_vel_callback, 10)
            self.dd_steering_subscription = self.create_subscription(Float32, 'driving_decision_steering_ang', self.vehicle_steering_callback, 10)
        else:
            self.sub_can_id451_ = self.create_subscription(DecodedCanMsg, "/can/from_can_bus/msgid451", self.can_id451_callback, 1)
            self.sub_can_id1281_ = self.create_subscription(DecodedCanMsg, "/can/from_can_bus/msgid1281", self.can_id1281_callback, 1)

        self.vehicle_pose_publisher = self.create_publisher(Pose2D, 'vehicle_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_vehicle_state)


        self.vehicle_pose_publisher = self.create_publisher(Pose2D, 'vehicle_pose', 10)
        

        self.timer = self.create_timer(0.1, self.publish_vehicle_state)

    def vehicle_vel_callback(self, msg):

        self.new_velocity = msg.linear.x
        
        #self.get_logger().info('I heard: vehicle velocity is =%f' % (self.new_velocity))
    
    def vehicle_steering_callback(self, msg):

        self.new_steering = msg.data
        
        #self.get_logger().info('I heard: vehicle steering is =%f' % (self.new_steering))  

    # CAN data callbacks
    def can_id451_callback(self, msg_id451):
        whl_spd_fl_ = msg_id451.sgnl_val[0]
        whl_spd_fr_ = msg_id451.sgnl_val[1]
        whl_spd_rl_ = msg_id451.sgnl_val[2]
        whl_spd_rr_ = msg_id451.sgnl_val[3]
        avg_wheel_speed = ((whl_spd_fl_ + whl_spd_fr_ + whl_spd_rl_ + whl_spd_rr_) / 4) # RPM
        v_fzg_ = (avg_wheel_speed * 2 * math.pi * self.r_wheels_ * 60) / 1000 # KM/h
        self.new_velocity = v_fzg_ / 3.6 # m/s  

    def can_id1281_callback (self, msg_id1281):
        
        # current steering angles
        current_st_agl_front_ = msg_id1281.sgnl_val[0] / self.magic_navya_steering_factor_ # front steering angle in degrees
        current_st_agl_rear_ = msg_id1281.sgnl_val[1] / self.magic_navya_steering_factor_
        # steering angle in radians
        self.new_steering = current_st_agl_front_  * math.pi / 180 # front steering angle in radians


    def update_vehicle_state(self):
        """
        Aktualisiert den Zustand des Fahrzeugs mit dem Ackermann-Modell.
        """

        # Aktualisierung des Fahrzeugzustands mit Ackermannmodell
        self.vehicle_v = self.new_velocity                                                      # replace by real testing
        self.vehicle_yaw += self.vehicle_v / self.L * math.tan(self.new_steering) * self.dt     # replace by real testing
        
        self.vehicle_x += self.vehicle_v * math.cos(self.vehicle_yaw) * self.dt
        self.vehicle_y += self.vehicle_v * math.sin(self.vehicle_yaw) * self.dt
        

    
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