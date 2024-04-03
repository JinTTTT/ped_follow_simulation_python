import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class PedestrianPositionPub(Node):
    def __init__(self):
        super().__init__('pedestrian_pos_pub_node')

        
        self.vehicle_pose_subscription =  self.create_subscription(Pose2D, 'vehicle_pose',  self.vehicle_pose_callback, 10)

        self.ped_pose_publisher = self.create_publisher(Pose2D, 'pedestrian_pos', 10)
        self.ped_x = 1.5  # 初始化行人x坐标为5
        self.ped_y = 2.0
        self.timer = self.create_timer(0.1, self.publish_position)

    def vehicle_pose_callback(self, msg):

        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_yaw = msg.theta
        
        #self.get_logger().info('I heard: vehicle pose is x=%f, y=%f, yaw=%f' % (msg.x, msg.y, msg.theta))  

    def publish_position(self):
        pedestrian_position = Pose2D()
        pedestrian_position.x = self.ped_x # 设置行人x坐标为5
        pedestrian_position.y = self.ped_y # 设置行人y坐标为0
        pedestrian_position.theta = 0.0  # 方向设置为0

        # 更新行人的x坐标，每次减少0.4，直到达到或低于1
        if self.ped_y > 0.0:
            self.ped_y -= 0.02
        else:
            self.ped_y = 0.0 # 一旦到达1，就保持不变

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
