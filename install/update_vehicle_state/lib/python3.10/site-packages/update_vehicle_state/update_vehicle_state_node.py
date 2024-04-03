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
        self.m_v = 1020.0  # [kg] Vorderachslast
        self.m_h = 1380.0  # [kg] Hinterachslast
        self.m = self.m_v + self.m_h  # [kg] Gesamtgewicht des Fahrzeugs
        self.L = 2.9  # [m] Radstand
        self.Spurweite = 1.895  # [m] Spurweite
        self.max_vel = 25  # [km/h] Höchstgeschwindigkeit

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 80 * np.pi / 180

        self.dt = 0.1 

        self.new_velocity = 0.0
        self.new_steering = 0.0
        
        self.dd_vel_subscription = self.create_subscription(Twist, 'driving_decision_vel' ,self.vehicle_vel_callback, 10)
        self.dd_steering_subscription =  self.create_subscription(Float32, 'driving_decision_steering_ang',  self.vehicle_steering_callback, 10)

        self.vehicle_pose_publisher = self.create_publisher(Pose2D, 'vehicle_pose', 10)
        

        self.timer = self.create_timer(0.1, self.publish_vehicle_state)

    def vehicle_vel_callback(self, msg):

        self.new_velocity = msg.linear.x
        
        #self.get_logger().info('I heard: vehicle velocity is =%f' % (self.new_velocity))
    
    def vehicle_steering_callback(self, msg):

        self.new_steering = msg.data
        
        #self.get_logger().info('I heard: vehicle steering is =%f' % (self.new_steering))    
        
    def update_vehicle_state(self):
        """
        Aktualisiert den Zustand des Fahrzeugs mit dem Ackermann-Modell.
        """

        # Aktualisierung des Fahrzeugzustands mit Ackermannmodell
        self.vehicle_v = self.new_velocity
        self.vehicle_x += self.vehicle_v * math.cos(self.vehicle_yaw) * self.dt
        self.vehicle_y += self.vehicle_v * math.sin(self.vehicle_yaw) * self.dt
        self.vehicle_yaw += self.vehicle_v / self.L * math.tan(self.new_steering) * self.dt

    
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