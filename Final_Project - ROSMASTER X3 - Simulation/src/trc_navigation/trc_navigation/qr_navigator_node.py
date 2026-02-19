#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

# ==============================
# CONFIGURATION MISSION
# ==============================
# J'ai ajouté un type "ACTION" ou "DEPOSIT" pour plus de clarté
MISSION = [
    {"name": "zone1",    "x": 1.0348, "y": -0.8711, "yaw": -0.4288, "duration": 4.0},
    {"name": "deposit1", "x": 0.8851, "y": -1.4041, "yaw": -0.4988, "duration": 2.0},
    {"name": "zone2",    "x": 1.4400, "y": -0.5900, "yaw": -0.0700, "duration": 4.0},
    {"name": "deposit1", "x": 0.8851, "y": -1.4041, "yaw": -0.4988, "duration": 2.0},
    {"name": "zone3",    "x": 1.3300, "y": 0.2200, "yaw": -0.1500, "duration": 4.0},
    {"name": "deposit1", "x": 0.8851, "y": -1.4041, "yaw": -0.4988, "duration": 2.0},
    {"name": "initial",  "x": 0.0233, "y": -0.0720, "yaw": -0.6667, "duration": 0.0},
]

class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_step = 0
        
        self.get_logger().info(" Algorithme de Compétition (Simulation) prêt")

        # Attente du serveur Nav2
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(" Attente de Nav2...")

        # Lancement de la première étape
        self.execute_step()

    def execute_step(self):
        if self.current_step >= len(MISSION):
            self.get_logger().info(" MISSION TERMINÉE AVEC SUCCÈS !")
            return

        target = MISSION[self.current_step]
        self.get_logger().info(f" ÉTAPE {self.current_step + 1}/{len(MISSION)} : {target['name']}")

        # Préparation du Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target["x"]
        goal_msg.pose.pose.position.y = target["y"]

        q = quaternion_from_euler(0, 0, target["yaw"])
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # Envoi de l'ordre à Nav2
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f" Destination {MISSION[self.current_step]['name']} refusée par Nav2 !")
            return

        self.get_logger().info(" Trajectoire validée par Nav2")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        # Status 4 = SUCCEEDED
        if status == 4:
            target = MISSION[self.current_step]
            self.get_logger().info(f" Arrivé à destination : {target['name']}")
            
            # --- SIMULATION DU RAMASSAGE / DÉPÔT ---
            if target['duration'] > 0:
                self.get_logger().info(f" Action en cours ({target['duration']}s)...")
                # On utilise un timer One-Shot pour la simulation
                self.timer = self.create_timer(target['duration'], self.after_action_callback)
            else:
                self.after_action_callback()
        else:
            self.get_logger().error(f" Échec de navigation (Status: {status})")

    def after_action_callback(self):
        # On détruit le timer pour éviter les boucles infinies
        if hasattr(self, 'timer'):
            self.timer.cancel()
            self.destroy_timer(self.timer)
        
        self.get_logger().info("✅ Action terminée.")
        self.current_step += 1
        self.execute_step()

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" Arrêt d'urgence demandé.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

