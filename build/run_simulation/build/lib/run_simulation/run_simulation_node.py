import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np


class RunSimulation(Node):
    def __init__(self):
        super().__init__('run_simulation_node')

        # 初始化用于存储数据的列表
        self.ped_x_list = []
        self.ped_y_list = []
        self.ped_yaw_list = []
        self.vel_list = []
        self.steering_list = []
        self.vehicle_x_list = []
        self.vehicle_y_list = []
        self.vehicle_yaw_list = [] 

        self.pedestrian_pos_subscription = self.create_subscription(Pose2D, 'pedestrian_pos', self.pedestrian_pos_callback, 10)
        self.dd_vel_subscription = self.create_subscription(Twist, 'driving_decision_vel' ,self.vehicle_vel_callback, 10)
        self.dd_steering_subscription =  self.create_subscription(Float32, 'driving_decision_steering_ang',  self.vehicle_steering_callback, 10)
        self.vehicle_pose_subscription =  self.create_subscription(Pose2D, 'vehicle_pose',  self.vehicle_pose_callback, 10)

        self.timer = self.create_timer(10.0, self.stop_simulation)
        
    def pedestrian_pos_callback(self, msg):

        self.ped_x_list.append(msg.x)
        self.ped_y_list.append(msg.y)
        self.ped_yaw_list.append(msg.theta)
        #self.get_logger().info('I heard: x=%f, y=%f, yaw = %f' % (msg.x, msg.y, msg.theta))
        
    def vehicle_vel_callback(self, msg):

        self.vel_list.append(msg.linear.x)      
        #self.get_logger().info('I heard: vehicle velocity is =%f' % (msg.linear.x))
    
    def vehicle_steering_callback(self, msg):

        steering_deg = msg.data * 180/ np.pi
        self.steering_list.append(steering_deg) 
        #self.get_logger().info('I heard: vehicle steering is =%f' % (msg.data))

    def vehicle_pose_callback(self, msg):

        self.vehicle_x_list.append(msg.x)
        self.vehicle_y_list.append(msg.y)
        self.vehicle_yaw_list.append(msg.theta)       
        #self.get_logger().info('I heard: vehicle pose is x=%f, y=%f, yaw=%f' % (msg.x, msg.y, msg.theta))  

    def stop_simulation(self):

        self.get_logger().info('10 seconds have passed. Stopping simulation and visualizing data.')
        self.visualize_data()
        self.timer.cancel()  # 停止计时器以结束仿真

    def visualize_data(self):
        plt.figure(figsize=(10, 8))

        # 绘制车辆和行人路径
        plt.subplot(2, 2, 1)
        plt.plot(self.vehicle_x_list, self.vehicle_y_list, 'k-', linewidth=1, label='Vehicle Path')
        plt.plot(self.ped_x_list, self.ped_y_list, 'b.', markersize=4, label='Pedestrian Path')
        plt.scatter(self.ped_x_list[-1], self.ped_y_list[-1], color='g', s=64, label='Current Pedestrian Position')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.title('Simulation Path')
        plt.legend(loc='upper left')
        plt.axis('equal')

        # 绘制转向角度
        plt.subplot(2, 2, 3)
        plt.plot(self.steering_list, 'b-', linewidth=1, label='Steering Angle')
        plt.axhline(y=30, color='r', linestyle='--', linewidth=1, label='Max Steering Angle')
        plt.axhline(y=-30, color='r', linestyle='--', linewidth=1, label='Min Steering Angle')
        plt.xlabel('Time [s]')
        plt.ylabel('Steering Angle [deg]')
        plt.title('Steering Angle Over Time')
        plt.legend(loc='best')

        # 绘制速度
        plt.subplot(2, 2, 4)
        plt.plot(self.vel_list, 'b-', linewidth=1, label='Velocity')
        plt.axhline(y=2, color='r', linestyle='--', linewidth=1, label='Max Velocity')  # v_max converted to m/s
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.title('Velocity Over Time')
        plt.legend(loc='best')

        plt.tight_layout()
        plt.show()
  

        
def main(args=None):
    rclpy.init(args=args)
    run_simulation_node = RunSimulation()
    rclpy.spin(run_simulation_node)
    run_simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()