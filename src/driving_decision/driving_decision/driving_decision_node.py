import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D, Twist
from std_msgs.msg import Float32
import math


class DrivingDecision(Node):

    def __init__(self):
        """
        Use this code to get the new driving command: desired driving velocity and desired steering angle.
        """
        
        super().__init__('driving_decision_node')
        self.ped_x = 0.0                        # pedstrian x position relative to the vehicle [m]
        self.ped_y = 0.0                        # pedstrian y position relative to the vehicle [m]                       
        
        self.vehicle_x = 0.0                    # vehicle x position global [m]        
        self.vehicle_y = 0.0                    # vehicle y position global [m]         
        self.vehicle_yaw = 0.0                  # vehicle yaw angle [rad]
        
        self.desired_vehicle_v = 0.0            # desired vehicle velocity [m/s]
        self.desired_steering = 0.0             

        # set the vehicle parameters
        self.v_max = 2.0                        # maximum velocity [m/s]  
        self.acc_max = 2.0                      # maximum acceleration [m/s^2]
        self.track_distance = 1.0               # set track distance to keep with the pedestrian [m]
        self.steering_range_max = math.pi / 6   # maximum steering angle [rad]
        self.steering_ang_vel_max = 15          # maximum steering angle change velocity [deg/s]

        
        self.last_velocity = 0.0                # last step vehicle velocity [m/s]
        self.last_delta = 0.0                   # last step steering angle [rad]    

        self.dt = 0.1                           # time step for simulation or real test [s]  need to change before real test
        
        self.pedestrian_pos_subscription = self.create_subscription(Pose2D, 'pedestrian_pos', self.pedestrian_pos_callback, 1)
        self.vehicle_pose_subscription =  self.create_subscription(Pose2D, 'vehicle_pose',  self.vehicle_pose_callback, 1)

        self.dd_vel_publisher = self.create_publisher(Twist, 'driving_decision_vel', 1)
        self.dd_steering_ang_publisher =  self.create_publisher(Float32, 'driving_decision_steering_ang', 1)

        self.timer = self.create_timer(self.dt, self.publish_driving_decision)

    def pedestrian_pos_callback(self, msg):

        self.ped_x = msg.x
        self.ped_y = msg.y
        #self.get_logger().info('I heard: x=%f, y=%f, z=%f' % (self.ped_x, self.ped_y, self.ped_yaw))
    
    def vehicle_pose_callback(self, msg):

        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_yaw = msg.theta
        #self.get_logger().info('I heard: vehicle pose is x=%f, y=%f, yaw=%f' % (msg.x, msg.y, msg.theta))  

        
    def steering_control(self):
        """
        Use this code to calculate the desired steering angle.
        How: based on the angle differenc between the vehicle's yaw angle and the angle from the vehicle to the pedestrian.
        
        """
        # define control parameter
        K_yaw =  0.8                                # P-controller for yaw angle control
        
        yaw_error = math.atan2(self.ped_y, self.ped_x)
        delta = K_yaw * yaw_error
        delta = max(min(delta, self.steering_range_max), -self.steering_range_max)
        delta_change = delta - self.last_delta
        max_delta_change = math.radians(self.steering_ang_vel_max) * self.dt
        
        if abs(delta_change) > max_delta_change:
            delta_change = math.copysign(max_delta_change, delta_change)
        
        new_delta = self.last_delta + delta_change
        new_delta = max(min(new_delta, self.steering_range_max), -self.steering_range_max)

        self.last_delta = new_delta
        return new_delta
    

    def velocity_control(self):
        """
        Use this code to calculate the desired vehicle velocity.
        How: Use cascade control, first control the distance between the vehicle and the pedestrian, then control the vehicle velocity.

        """
        # define control parameter
        Kp_distance = 1.0  
        Kp_vel = 1.0  

        distance_diff = self.ped_x - self.track_distance

        if distance_diff <= 0:
            velocity_ref = 0
        else:
            velocity_ref = Kp_distance * distance_diff  
            velocity_ref = min(velocity_ref, self.v_max)  
        
        velocity_diff = velocity_ref - self.last_velocity
        acceleration_command = Kp_vel * velocity_diff  

        acceleration_command = min(acceleration_command, self.acc_max * self.dt)  
        new_velocity = self.last_velocity + acceleration_command * self.dt
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
        #self.get_logger().info('I published: vehicle velocity is =%f' % (self.desired_vehicle_v))

def main(args=None):
    rclpy.init(args=args)
    driving_decision_node_real = DrivingDecision()
    rclpy.spin(driving_decision_node_real)
    driving_decision_node_real.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()