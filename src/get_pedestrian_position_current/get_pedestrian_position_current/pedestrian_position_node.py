import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math

class PedestrianPositionPub(Node):
    """
    Use this code to create a sequence of pedestrian position and publish them in the vehicle coordinate.
    Assume that the publish data is obtained from the camera sensor.

    """
    def __init__(self):
        super().__init__('pedestrian_pos_pub_node')

        self.vehicle_x = 0.0            # vehicle x position [m]
        self.vehicle_y = 0.0            # vehicle y position [m]
        self.vehicle_yaw = 0.0          # vehicle yaw angle [rad]   

        self.global_ped_x = 2.0         # global pedestrian x position [m]
        self.global_ped_y = 2.0         # global pedestrian y position [m]  

        self.dt = 0.1                   # time step [s]
        
        self.vehicle_pose_subscription =  self.create_subscription(Pose2D, 'vehicle_pose',  self.vehicle_pose_callback, 1)
        self.ped_pose_publisher = self.create_publisher(Pose2D, 'pedestrian_pos', 1)
        self.timer = self.create_timer(self.dt, self.publish_position)

    def vehicle_pose_callback(self, msg):

        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_yaw = msg.theta
        
    def publish_position(self):
        """
        To fullfill the real test, the obtained pedestrian position from sensor should be in the vehicle coordinate system.
        Use this funktion get the relative position of the pedestrian base on the transform matrix.  

        """

        translated_ped_x = self.global_ped_x - self.vehicle_x
        translated_ped_y = self.global_ped_y - self.vehicle_y

        ped_rel_x = translated_ped_x * math.cos(self.vehicle_yaw) + translated_ped_y * math.sin(self.vehicle_yaw)
        ped_rel_y = -translated_ped_x * math.sin(self.vehicle_yaw) + translated_ped_y * math.cos(self.vehicle_yaw)

        pedestrian_position = Pose2D()
        pedestrian_position.x = ped_rel_x 
        pedestrian_position.y = ped_rel_y 
        pedestrian_position.theta = 0.0  
       
        self.global_ped_x += 0.12
        self.ped_pose_publisher.publish(pedestrian_position)
        # self.get_logger().info(f'Publishing: x={pedestrian_position.x}, y={pedestrian_position.y}')

def main(args=None):
    rclpy.init(args=args)
    pedestrian_position_pub_node = PedestrianPositionPub()
    rclpy.spin(pedestrian_position_pub_node)
    pedestrian_position_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
