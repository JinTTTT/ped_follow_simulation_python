import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np
import math


class RunSimulation(Node):
    def __init__(self):
        super().__init__('run_simulation_node')

        self.ped_x = 0.0
        self.ped_y = 0.0
        

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        

        # 初始化用于存储数据的列表
        self.ped_x_list = []
        self.ped_y_list = []
    
        self.vel_list = []
        self.steering_list = []
        self.vehicle_x_list = []
        self.vehicle_y_list = []
        self.vehicle_yaw_list = [] 

        self.pedestrian_pos_subscription = self.create_subscription(Pose2D, 'pedestrian_pos', self.pedestrian_pos_callback, 10)
        self.dd_vel_subscription = self.create_subscription(Twist, 'driving_decision_vel' ,self.vehicle_vel_callback, 10)
        self.dd_steering_subscription =  self.create_subscription(Float32, 'driving_decision_steering_ang',  self.vehicle_steering_callback, 10)
        self.vehicle_pose_subscription =  self.create_subscription(Pose2D, 'vehicle_pose',  self.vehicle_pose_callback, 10)

        self.timer = self.create_timer(20.0, self.stop_simulation)
        
    def pedestrian_pos_callback(self, msg):

        # 计算行人的全局坐标
        global_ped_x = self.vehicle_x + msg.x * math.cos(self.vehicle_yaw) - msg.y * math.sin(self.vehicle_yaw)
        global_ped_y = self.vehicle_y + msg.x * math.sin(self.vehicle_yaw) + msg.y * math.cos(self.vehicle_yaw)

        self.ped_x_list.append(global_ped_x)
        self.ped_y_list.append(global_ped_y)
        
        #self.get_logger().info('I heard: x=%f, y=%f, yaw = %f' % (msg.x, msg.y, msg.theta))
        
    def vehicle_vel_callback(self, msg):

        self.vel_list.append(msg.linear.x)      
        #self.get_logger().info('I heard: vehicle velocity is =%f' % (msg.linear.x))
    
    def vehicle_steering_callback(self, msg):

        steering_deg = msg.data * 180/ np.pi
        self.steering_list.append(steering_deg) 
        #self.get_logger().info('I heard: vehicle steering is =%f' % (msg.data))

    def vehicle_pose_callback(self, msg):

        # 更新车辆的全局位置和朝向
        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_yaw = msg.theta

        self.vehicle_x_list.append(msg.x)
        self.vehicle_y_list.append(msg.y)
        self.vehicle_yaw_list.append(msg.theta)       
        #self.get_logger().info('I heard: vehicle pose is x=%f, y=%f, yaw=%f' % (msg.x, msg.y, msg.theta))  

    def stop_simulation(self):

        self.get_logger().info('10 seconds have passed. Stopping simulation and visualizing data.')
        self.visualize_data()
        self.timer.cancel()  # 停止计时器以结束仿真

    def visualize_data(self):

        plt.ion()
        figure = plt.figure(figsize=(10, 8))
        i = 1
        while True:
            plt.clf()  # 清除当前图形

            if i > len(self.vehicle_x_list):  # 如果i超过了数据长度，结束循环
                break

            # 绘制车辆和行人路径
            ax1 = figure.add_subplot(2, 2, 1)
            ax1.plot(self.vehicle_x_list[:i], self.vehicle_y_list[:i], 'k-', linewidth=1, label='Vehicle Path')
            ax1.plot(self.ped_x_list[:i], self.ped_y_list[:i], 'b.', markersize=4, label='Pedestrian Path')
            if i > 1:  # 确保至少有一个点
                ax1.scatter(self.ped_x_list[i-1], self.ped_y_list[i-1], color='g', s=64, label='Current Pedestrian Position')
                ax1.scatter(self.vehicle_x_list[i-1], self.vehicle_y_list[i-1], color='g', s=64, label='Current Vehicle Position')
                
            ax1.set_xlabel('X Position [m]')
            ax1.set_ylabel('Y Position [m]')
            ax1.set_title('Simulation Path')
            ax1.legend(loc='upper left')
            ax1.axis('equal')

            # 绘制转向角度
            ax2 = figure.add_subplot(2, 2, 3)
            ax2.plot(self.steering_list[:i], 'b-', linewidth=1, label='Steering Angle')
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Steering Angle [deg]')
            ax2.set_title('Steering Angle Over Time')
            ax2.legend(loc='best')

            # 绘制速度
            ax3 = figure.add_subplot(2, 2, 4)
            ax3.plot(self.vel_list[:i], 'b-', linewidth=1, label='Velocity')
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Velocity [m/s]')
            ax3.set_title('Velocity Over Time')
            ax3.legend(loc='best')

            plt.tight_layout()
            plt.draw()
            plt.pause(0.1)  # 短暂暂停，以便观察到更新的图形

            i += 1  # 更新循环变量

        plt.ioff()  # 关闭交互模式

  

        
def main(args=None):
    rclpy.init(args=args)
    run_simulation_node = RunSimulation()
    rclpy.spin(run_simulation_node)
    run_simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()