import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import math


class DrivingDecision(Node):

    def __init__(self):
        super().__init__('driving_decision_node')
        self.ped_x = 0.0
        self.ped_y = 0.0
        self.ped_yaw = 0.0

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        self.last_delta = 0.0

        self.desired_vehicle_v = 0.0
        self.desired_steering = 0.0

        self.v_max = 2.0  # 假设的最大速度
        self.acc_max = 2.0  # 假设的最大加速度
        self.track_distance = 1.0  # 与行人保持的距离
        self.last_velocity = 0.0

        self.dt = 0.1 
        
        self.pedestrian_pos_subscription = self.create_subscription(Pose2D, 'pedestrian_pos', self.pedestrian_pos_callback, 10)
        self.vehicle_pose_subscription =  self.create_subscription(Pose2D, 'vehicle_pose',  self.vehicle_pose_callback, 10)

        self.dd_vel_publisher = self.create_publisher(Twist, 'driving_decision_vel', 10)
        self.dd_steering_ang_publisher =  self.create_publisher(Float32, 'driving_decision_steering_ang', 10)


        self.timer = self.create_timer(0.1, self.publish_driving_decision)

    def pedestrian_pos_callback(self, msg):

        self.ped_x = msg.x
        self.ped_y = msg.y
        self.ped_yaw = msg.theta
        #self.get_logger().info('I heard: x=%f, y=%f, z=%f' % (self.ped_x, self.ped_y, self.ped_yaw))
    
    def vehicle_pose_callback(self, msg):

        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_yaw = msg.theta
        
        #self.get_logger().info('I heard: vehicle pose is x=%f, y=%f, yaw=%f' % (msg.x, msg.y, msg.theta))  

        
    def steering_control(self):
        """
        Diese Funktion berechnet den Lenkwinkel des Fahrzeugs basierend auf der
        relative Position zwischen Fahrzeug und Fußgänger.
        Es gibt hier zwei Regler für die Lenkungsregelung, Querabweichungsregelung
        und Gierwinkelsregelung.
        
        """
        # Parameter Definieren
        steering_range_max = math.pi / 6            # radiant, maximale Lenkungswinkel am Rad
        steering_ang_vel_max = 15                   # deg/s, maximale Lenksgeschwindigkeit
        K_yaw =  0.8                                # Gierwinkelsregelung Proportionalverstärkung
        
    
        yaw_error = math.atan2(self.ped_y, self.ped_x)
        delta = K_yaw * yaw_error

        delta = max(min(delta, steering_range_max), -steering_range_max)
        delta_change = delta - self.last_delta

        max_delta_change = math.radians(steering_ang_vel_max) * self.dt
        
        if abs(delta_change) > max_delta_change:
            delta_change = math.copysign(max_delta_change, delta_change)
        
        new_delta = self.last_delta + delta_change
        
        new_delta = max(min(new_delta, steering_range_max), -steering_range_max)

        self.last_delta = new_delta

        return new_delta
    

    def velocity_control(self):
        Kp_distance = 1.0  # P-regler für Abstandregelung
        Kp_vel = 1.0  # P-regler für Geschwindigkeitsregelung

        # Abstandsregelung basierend auf der x-Position des Fußgängers relativ zum Fahrzeug
        distance_diff = self.ped_x - self.track_distance

        # Wenn der Abstand kleiner oder gleich dem Sicherheitsabstand ist,
        # soll Geschwindigkeit = 0, vollbremsen
        if distance_diff <= 0:
            velocity_ref = 0
        else:
            velocity_ref = Kp_distance * distance_diff  
            velocity_ref = min(velocity_ref, self.v_max)  # Beschränkung der Geschwindigkeit auf v_max

        # Geschwindigkeitsregelung
        velocity_diff = velocity_ref - self.last_velocity
        acceleration_command = Kp_vel * velocity_diff  # P-regler für Geschwindigkeitsregelung

        # Beschleunigung begrenzen
        acceleration_command = min(acceleration_command, self.acc_max * self.dt)  # Beschleunigungsgrenze (maximal 2 m/s^2
        
        # aktualisieren die neue Geschwindigkeit
        new_velocity = self.last_velocity + acceleration_command * self.dt

        # Beschränkung der neuen Geschwindigkeit im Intervall [0, v_max]
        new_velocity = max(min(new_velocity, self.v_max), 0)

        self.last_velocity = new_velocity

        return new_velocity

    
    def publish_driving_decision(self):

        self.desired_steering = self.steering_control()
        self.desired_vehicle_v = self.velocity_control()
        
        vel_msg = Twist()
        vel_msg.linear.x = self.desired_vehicle_v
        self.dd_vel_publisher.publish(vel_msg)

        steering_msg = Float32()
        steering_msg.data = self.desired_steering
        self.dd_steering_ang_publisher.publish(steering_msg)
        #print('velocity is %f, steering is %f'%(new_velocity, desired_steering_angle))

def main(args=None):
    rclpy.init(args=args)
    driving_decision_node_real = DrivingDecision()
    rclpy.spin(driving_decision_node_real)
    driving_decision_node_real.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()