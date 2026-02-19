#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from yahboomcar_msgs.msg import Servo
import math

# ==============================================================================
# FONCTIONS UTILITAIRES
# ==============================================================================
def get_quaternion(yaw):
    """Convertit un angle Euler (Yaw) en Quaternion pour ROS 2 Nav2."""
    return {'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}

# ==============================================================================
# CONFIGURATION DE LA MISSION (10 CYCLES)
# ==============================================================================
MISSION = [
    {"name": "PICK_1",   "type": "PICK",    "x": 1.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_1",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_2",   "type": "PICK",    "x": 2.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_2",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_3",   "type": "PICK",    "x": 3.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_3",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_4",   "type": "PICK",    "x": 4.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_4",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_5",   "type": "PICK",    "x": 5.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_5",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_6",   "type": "PICK",    "x": 6.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_6",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_7",   "type": "PICK",    "x": 7.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_7",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_8",   "type": "PICK",    "x": 8.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_8",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_9",   "type": "PICK",    "x": 9.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_9",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "PICK_10",  "type": "PICK",    "x": 10.0, "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_10", "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},
    {"name": "FIN_MATCH", "type": "NONE",    "x": 0.0,  "y": 0.0,  "yaw": 0.0},
]

# ==============================================================================
# CLASSE PRINCIPALE : GESTIONNAIRE DE MISSION
# ==============================================================================
class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        self.servo_pub = self.create_publisher(Servo, '/servo', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.current_step = 0
        self.items_delivered = 0
        self.action_timer = None

        self.get_logger().info("🚀 MODE TEST ALGO (ATTENTE 4s) : PRÊT")

        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("En attente de Nav2...")

        self.send_next_goal()

    def move_servo(self, servoid, angle):
        """Prêt pour plus tard - Publie une commande de servo."""
        msg = Servo()
        msg.servoid, msg.angle = servoid, int(angle)
        self.servo_pub.publish(msg)

    # --------------------------------------------------------------------------
    # SEQUENCES TEMPORISÉES (REMPLACE LES SERVOS POUR LE TEST)
    # --------------------------------------------------------------------------
    def start_mechanical_action(self, action_type):
        """Simule une action mécanique par une attente de 4 secondes."""
        self.get_logger().info(f"⏳ Action {action_type} en cours (Attente 4s...)")
        
        # On crée un timer unique de 4 secondes
        self.action_timer = self.create_timer(4.0, self.finish_action)

    def finish_action(self):
        """Passe à l'étape suivante après l'attente."""
        if self.action_timer:
            self.action_timer.cancel()
            self.action_timer = None
        
        target_type = MISSION[self.current_step]['type']
        if target_type == "DEPOSIT":
            self.items_delivered += 1
            self.get_logger().info(f"✅ Dépôt {self.items_delivered}/10 terminé.")
        else:
            self.get_logger().info(f"✅ Collecte terminée.")

        self.current_step += 1
        self.send_next_goal()

    # --------------------------------------------------------------------------
    # GESTION NAVIGATION NAV2
    # --------------------------------------------------------------------------
    def send_next_goal(self):
        if self.current_step >= len(MISSION):
            self.get_logger().info("🏆 MISSION ACCOMPLIE !")
            return

        target = MISSION[self.current_step]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(target['x'])
        goal_msg.pose.pose.position.y = float(target['y'])
        
        q = get_quaternion(target['yaw'])
        goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w = q['z'], q['w']

        self.get_logger().info(f"📍 Navigation vers : {target['name']}")
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Cible refusée par Nav2 !")
            self.current_step += 1
            self.send_next_goal()
            return
        handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCESS
            target_type = MISSION[self.current_step]['type']
            if target_type != "NONE":
                self.start_mechanical_action(target_type)
            else:
                self.current_step += 1
                self.send_next_goal()
        else:
            self.get_logger().warn(f"Cible manquée (Status {status}), passage à la suivante.")
            self.current_step += 1
            self.send_next_goal()

# ==============================================================================
# EXECUTION
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
