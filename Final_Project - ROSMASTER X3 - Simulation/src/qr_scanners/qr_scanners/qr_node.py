import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar
import os
import csv
from datetime import datetime

class QRNode(Node):
    def __init__(self):
        super().__init__('qr_node')
        self.bridge = CvBridge()

        # Subscription à la caméra
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Subscription à l'odom pour la position
        self.robot_position = None
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Création du dossier data/images
        self.data_dir = os.path.join(os.getcwd(), 'data')
        self.images_dir = os.path.join(self.data_dir, 'images')
        os.makedirs(self.images_dir, exist_ok=True)

        # CSV
        self.csv_path = os.path.join(self.data_dir, 'qr_log.csv')
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'qr_text', 'x', 'y', 'z', 'yaw'])

    def odom_callback(self, msg: Odometry):
        # Récupère la position et orientation (yaw)
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Conversion quaternion → yaw
        import math
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.robot_position = (pos.x, pos.y, pos.z, yaw)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qrcodes = pyzbar.decode(cv_image)

        for qr in qrcodes:
            x, y, w, h = qr.rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            qr_text = qr.data.decode('utf-8')
            self.get_logger().info(f'Detected QR: {qr_text}')

            # Position actuelle du robot
            if self.robot_position is not None:
                robot_x, robot_y, robot_z, robot_yaw = self.robot_position
            else:
                robot_x, robot_y, robot_z, robot_yaw = 0.0, 0.0, 0.0, 0.0

            # Sauvegarde CSV
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([datetime.now(), qr_text, robot_x, robot_y, robot_z, robot_yaw])

            # Sauvegarde image
            img_name = f"qr_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            cv2.imwrite(os.path.join(self.images_dir, img_name), cv_image)

        cv2.imshow('QR Detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
