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
        K_lateral = 0.0                             # Querabweichungsregekung Proportionalverstärkung
        K_yaw =  0.8                                # Gierwinkelsregelung Proportionalverstärkung
        
        # 1. Querabweichungsregelung
        direction_vehicle_to_pedestrian = [self.ped_x - self.vehicle_x, self.ped_y - self.vehicle_y]  # RichtungsVektor: Fahrzeug zu Fußgänger
        direction_vehicle = [math.cos(self.vehicle_yaw), math.sin(self.vehicle_yaw)]  # RichtungsVektor: Fahrzeug selbst

        # Kreuzprodukt, um zu bestimmen, ob der Fußgänger links oder rechts vom Fahrzeug liegt
        cross_product = direction_vehicle_to_pedestrian[1] * direction_vehicle[0] - direction_vehicle_to_pedestrian[0] * direction_vehicle[1]

        # Berechnung des Querdifferenz (示例)
        dx = self.ped_x - self.vehicle_x
        dy = self.ped_y - self.vehicle_y
        # print('ped is %f, %f'%(self.ped_x, self.ped_y))
        # print('vehicl is %f, %f'%(self.vehicle_x, self.vehicle_y))
        
        if cross_product > 0:
            lateral_error = math.sqrt(dx**2 + dy**2) * math.cos(self.ped_yaw) * 1
        elif cross_product < 0:
            lateral_error = math.sqrt(dx**2 + dy**2) * math.cos(self.ped_yaw) * -1
        else:
            lateral_error = math.sqrt(dx**2 + dy**2) * math.cos(self.ped_yaw) * 0

        # print('abs error is', math.sqrt(dx**2 + dy**2))
        # print('lateral error is', lateral_error)
        # print('cos is', math.cos(self.ped_yaw))
        # Berechnung der Steuergröße (Lenkwinkel)
        delta_lateral = K_lateral * lateral_error

        # 2. Gierwinkelsregelung
        yaw_to_target = math.atan2(self.ped_y - self.vehicle_y, self.ped_x - self.vehicle_x)  # Berechnung des Gierwinkels vom Fahrzeug zum Fußgänger

        # Berechnung des Gierwinkelsdifferenz
        yaw_error = yaw_to_target - self.vehicle_yaw

        # Berechnung der Steuergröße (Lenkwinkel)
        delta_yaw = K_yaw * yaw_error

        # neuer Lenkwinkel = Lenkwinkel der Querabweichungsregelung + Lenkwinkel der Gierwinkelsregelung
        delta = delta_lateral + delta_yaw
        
        # Begrenzung des Lenkwinkel und der Lenkgeschwindigkeit
        delta = max(min(delta, steering_range_max), -steering_range_max)
        
        delta_change = delta - self.last_delta
        max_delta_change = math.radians(steering_ang_vel_max) * self.dt  # Umwandlung von Grad/s zu Radiant/s und Multiplikation mit dt

        # Lenkgeschwindigkeit begrenzen
        if abs(delta_change) > max_delta_change:
            delta_change = math.copysign(max_delta_change, delta_change)
        
        # Lenkwinkel aktualisieren und im gültigen Bereich halten
        new_delta = self.last_delta + delta_change
        
        return new_delta
    

    def velocity_control(self, last_velocity, v_max, acc_max, vehicle_x, vehicle_y, target_x, target_y, track_distance, dt):
        """
        Diese Funktion berechnet eine Zielgeschwindigkeit für das Fahrzeug,
        basierend auf den Positionsinformationen des Fahrzeugs und des vorausfahrenden Fußgängers.
        Die Zielgeschwindigkeit ermöglicht es dem Fahrzeug, dem Fußgänger bei gleichzeitiger Einhaltung eines sicheren Abstands zu folgen.
        """

        Kp_distance = 0.5  # P-regler für Abstandregelung
        Kp_vel = 2.0  # P-regler für Geschwindigkeitsregelung

        # Äußere Schleife: Abstandsregelung
        distance_to_target = math.sqrt((vehicle_x - target_x)**2 + (vehicle_y - target_y)**2)
        distance_diff = distance_to_target - track_distance

        # Wenn der Abstand zum Fußgänger kleiner als der Sicherheitsabstand,
        # soll_geschwindigkeit = 0, vollbremsen
        if distance_diff <= 0:
            velocity_ref = 0
        else:
            velocity_ref = Kp_distance * distance_diff  
            velocity_ref = min(velocity_ref, v_max)    # Beschränkung der Geschwindigkeit

        # Innere Schleife: Geschwindigkeitsregelung
        velocity_diff = velocity_ref - last_velocity
        acceleration_command = Kp_vel * velocity_diff  # P-regler für Geschwindigkeitsregelung

        # Beschränkung der Beschleunigung
        acceleration_command = min(acceleration_command, acc_max)
        
        # aktualisieren die neue Geschwindigkeit
        new_velocity = last_velocity + acceleration_command * dt

        # Beschränkung der neue Geschwindigkeit im Intervall [0, v_max]
        new_velocity = max(min(new_velocity, v_max), 0)

        return new_velocity, velocity_ref
    
    def publish_driving_decision(self):

        desired_steering_angle = self.steering_control()
        

        new_velocity, _ = self.velocity_control(
            self.last_velocity, self.v_max, self.acc_max,
            self.vehicle_x, self.vehicle_y, self.ped_x, self.ped_y,
            self.track_distance, self.dt
        )

        self.last_delta = desired_steering_angle
        self.last_velocity = new_velocity
        
        vel_msg = Twist()
        vel_msg.linear.x = new_velocity
        self.dd_vel_publisher.publish(vel_msg)

        steering_msg = Float32()
        steering_msg.data = desired_steering_angle
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