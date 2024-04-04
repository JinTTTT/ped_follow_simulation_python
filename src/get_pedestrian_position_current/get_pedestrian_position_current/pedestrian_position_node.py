import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math

class PedestrianPositionPub(Node):
    def __init__(self):
        super().__init__('pedestrian_pos_pub_node')

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0

        self.global_ped_x = 2.0  # 初始化行人x坐标为5
        self.global_ped_y = 2.0

        
        self.vehicle_pose_subscription =  self.create_subscription(Pose2D, 'vehicle_pose',  self.vehicle_pose_callback, 10)

        self.ped_pose_publisher = self.create_publisher(Pose2D, 'pedestrian_pos', 10)
        
        self.timer = self.create_timer(0.1, self.publish_position)

    def vehicle_pose_callback(self, msg):

        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_yaw = msg.theta
        
        #self.get_logger().info('I heard: vehicle pose is x=%f, y=%f, yaw=%f' % (msg.x, msg.y, msg.theta))  

    def publish_position(self):

        translated_ped_x = self.global_ped_x - self.vehicle_x
        translated_ped_y = self.global_ped_y - self.vehicle_y

        ped_rel_x = translated_ped_x * math.cos(self.vehicle_yaw) + translated_ped_y * math.sin(self.vehicle_yaw)
        ped_rel_y = -translated_ped_x * math.sin(self.vehicle_yaw) + translated_ped_y * math.cos(self.vehicle_yaw)


        pedestrian_position = Pose2D()

        pedestrian_position.x = ped_rel_x # 设置行人x坐标为5
        pedestrian_position.y = ped_rel_y # 设置行人y坐标为0
        pedestrian_position.theta = 0.0  # 方向设置为0

        # 更新行人的x坐标，每次减少0.4，直到达到或低于1
        
        self.global_ped_x += 0.12
        
        self.ped_pose_publisher.publish(pedestrian_position)
        self.get_logger().info(f'Publishing: x={pedestrian_position.x}, y={pedestrian_position.y}')

    

def main(args=None):
    rclpy.init(args=args)
    pedestrian_position_pub_node = PedestrianPositionPub()
    rclpy.spin(pedestrian_position_pub_node)
    pedestrian_position_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
