import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D

import os
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory


class PedestrianPositionPub(Node):
    def __init__(self):
        # pedestrian_pos_pub_node is the name of this node
        super().__init__('pedestrian_pos_pub_node')
        self.ped_pose_publisher = self.create_publisher(Pose2D, 'pedestrian_pos', 10)

        self.timer = self.create_timer(0.1, self.publish_position)
        self.resampled_x, self.resampled_y = self.get_pedestrian_position()
        self.pedestrian_yaw = self.calculate_pedestrian_yaw(self.resampled_x, self.resampled_y)  # 计算方向
        self.i = 0

    def get_pedestrian_position(self):
        resampled_x, resampled_y = self.data_extraction_pedestrians()
        return resampled_x, resampled_y
    
    def data_extraction_pedestrians(self):
        """
        Diese Funktion dient dazu, die benötigene X und Y Position des
        Fußgängers von der aufgenommene Daten zu auslesen. Außerdem führt auch die
        Vorverarbeitung der Daten durch, wie z.B erneute Abtastung, erneute
        Interpolation.
        """

        package_name = 'get_pedestrian_position_current'
        package_share_directory = get_package_share_directory(package_name)
        json_file_path = os.path.join(package_share_directory, 'resource', 'AVL_Example.json')

        with open(json_file_path, 'r') as file:
            json_data = json.load(file)

        # Zur Speicherung der X-, Y- und Z-Koordinaten des Fußgängers
        data_x, data_y, data_z = [], [], []

        # Speicherung für jedes Zeitstempels in den JSON-Daten
        for timestamp in json_data.keys():
            keypoints = json_data[timestamp]

            # Suchen des Keypoints mit Keypoint = 0 und Extrahieren seiner Koordinaten
            for keypoint in keypoints:
                if keypoint['keypoint'] == 0:
                    data_x.append(keypoint['x'])
                    data_y.append(keypoint['y'])
                    data_z.append(keypoint['z'])
                    break

        # Umwandeln der Koordinatenwerte, X/Z/Y zu X/Y/Z, Umrechnung der Einheit
        X = np.array(data_x) / 1000.0  # [mm] -> [m]
        Y = np.array(data_z) / 1000.0  # [mm] -> [m]

        # Verschieben die ursprünliche Koordinaten zu (0,0)
        X = X + 1.0  # Beginnt bei x=0/y=0
        Y = Y - 8.0  # Beginnt bei x=0/y=0

        # Daten verarbeiten 1.Neuabtastung von Rohdaten mit gewünschter Abtastfrequenz, 2.Entfernung ungültiger Daten (Inf), 3.Erneute Interpolation -- Daten glätten
        original_freq = 60  # Hz, 1/60 s für jede Schritt, die originale Frequenz der Daten
        desired_freq = 10   # Hz, 1/10 s für jede Schritt(Simulation), die gewünschte Frequenz

        # Neuabtastung, Entfernung ungültiger Daten (Inf)
        resampled_x, resampled_y = self.resample_position(X, Y, original_freq, desired_freq)

    
        return resampled_x, resampled_y
    
    def resample_position(self, X, Y, original_freq, desired_freq):
        # Entfernung ungültiger Daten (Inf)
        valid_idx = ~(np.isinf(X) | np.isinf(Y))
        X = X[valid_idx]
        Y = Y[valid_idx]

        # Neuabtastung und Interpolation
        original_time_interval = 1 / original_freq
        desired_time_interval = 1 / desired_freq
        number_of_data = len(X)
        original_time_vector = np.arange(0, number_of_data * original_time_interval, original_time_interval)
        target_time = np.arange(0, original_time_vector[-1], desired_time_interval)

        f_x = interp1d(original_time_vector, X, kind='cubic')
        f_y = interp1d(original_time_vector, Y, kind='cubic')

        resampled_x = f_x(target_time)
        resampled_y = f_y(target_time)

        return resampled_x, resampled_y
    
    
    
    def calculate_pedestrian_yaw(self, resampled_x, resampled_y):
        """
        Berechnet die Bewegungsrichtung des Fußgängers an jedem Punkt.
        """
        
        pedestrian_yaw = np.zeros_like(resampled_x)

        # 从第二个点开始计算每个点的方向
        for j in range(1, len(resampled_x)):
            dx = resampled_x[j] - resampled_x[j-1]
            dy = resampled_y[j] - resampled_y[j-1]
            pedestrian_yaw[j] = np.arctan2(dy, dx)

        # 第一个点的方向可以设为第二个点的方向，或者根据实际情况处理
        pedestrian_yaw[0] = pedestrian_yaw[1]

        return pedestrian_yaw
    
    def publish_position(self):
        if self.i < len(self.resampled_x) and self.i < len(self.resampled_y):
            pedestrian_position_current = Pose2D()
            pedestrian_position_current.x = float(self.resampled_x[self.i])
            pedestrian_position_current.y = float(self.resampled_y[self.i])
            pedestrian_position_current.theta = self.pedestrian_yaw[self.i]
          
            self.ped_pose_publisher.publish(pedestrian_position_current)
            #self.get_logger().info('I heard: x=%f, y=%f, yaw=%f' % (pedestrian_position_current.x, pedestrian_position_current.y, pedestrian_position_current.theta))
        
            self.i += 1
        else:
            #self.get_logger().info('Pedestrain stopped!')
            pedestrian_position_current = Pose2D()
            pedestrian_position_current.x = float(self.resampled_x[-1])
            pedestrian_position_current.y = float(self.resampled_y[-1])
            pedestrian_position_current.theta = self.pedestrian_yaw[-1]
          
            self.ped_pose_publisher.publish(pedestrian_position_current)
            #self.get_logger().info('I heard: x=%f, y=%f, yaw=%f' % (pedestrian_position_current.x, pedestrian_position_current.y, pedestrian_position_current.theta))
        

def main(args = None):
    rclpy.init(args=args)
    pedestrain_position_pub_node = PedestrianPositionPub()
    rclpy.spin(pedestrain_position_pub_node)
    pedestrain_position_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

