#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Messages ROS
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

# Conversion ROS Image -> OpenCV
from cv_bridge import CvBridge

# Vision
import cv2
from pyzbar import pyzbar

# Utilitaires
import os
import csv
from datetime import datetime
import math

class QRNode(Node):
    def __init__(self):
        super().__init__('qr_node')

        # ============================
        # Initialisation
        # ============================
        self.bridge = CvBridge()
        self.robot_position = None
        
        # Liste pour mémoriser les QR déjà loggés et éviter les doublons
        self.logged_qrs = set() 

        # ============================
        # PARAMÈTRES ROS
        # ============================
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('show_image', False)

        image_topic = self.get_parameter('image_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.show_image = self.get_parameter('show_image').value

        # ============================
        # GESTION DES DOSSIERS (Auto-création)
        # ============================
        # On utilise un chemin clair dans ton home
        self.data_dir = os.path.expanduser('~/trc_ws/qr_results')
        self.images_dir = os.path.join(self.data_dir, 'images')

        # Création récursive des dossiers si inexistants
        if not os.path.exists(self.images_dir):
            os.makedirs(self.images_dir)
            self.get_logger().info(f'Création du dossier : {self.images_dir}')

        # ============================
        # GESTION DU CSV (Auto-création)
        # ============================
        self.csv_path = os.path.join(self.data_dir, 'qr_log.csv')
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # En-tête du fichier
                writer.writerow(['timestamp', 'qr_text', 'x', 'y', 'z', 'yaw'])
            self.get_logger().info(f'Création du fichier CSV : {self.csv_path}')

        # ============================
        # SUBSCRIPTIONS
        # ============================
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info('QR Node prêt ! (Zéro doublons activé)')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Conversion simple quaternion vers yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_position = (pos.x, pos.y, pos.z, yaw)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        qrcodes = pyzbar.decode(cv_image)

        for qr in qrcodes:
            qr_text = qr.data.decode('utf-8')

            # --- GESTION DOUBLONS ---
            if qr_text in self.logged_qrs:
                continue # On ignore si déjà vu dans cette session
            
            # Si c'est un nouveau QR :
            self.logged_qrs.add(qr_text)
            self.get_logger().info(f'🆕 NOUVEAU QR DÉTECTÉ : {qr_text}')

            # Dessin sur l'image
            x, y, w, h = qr.rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Coordonnées
            rx, ry, rz, ryaw = self.robot_position if self.robot_position else (0.0, 0.0, 0.0, 0.0)

            # Sauvegarde CSV
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([datetime.now().isoformat(), qr_text, rx, ry, rz, ryaw])
            
            # Sauvegarde Image
            img_filename = f"qr_{qr_text.replace(' ', '_')[:10]}_{datetime.now().strftime('%H%M%S')}.png"
            cv2.imwrite(os.path.join(self.images_dir, img_filename), cv_image)
            self.get_logger().info(f'✅ Loggé dans le CSV et image sauvegardée.')

        if self.show_image:
            cv2.imshow('QR Detection', cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
