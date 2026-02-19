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
# CONFIGURATION DE LA MISSION (10 CYCLES - 4 MINUTES)
# Strategie : Points de passage precis, vitesse maximale.
# ==============================================================================
MISSION = [
    # --- CYCLE 1 ---
    {"name": "PICK_1",   "type": "PICK",    "x": 1.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_1",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 2 ---
    {"name": "PICK_2",   "type": "PICK",    "x": 2.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_2",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 3 ---
    {"name": "PICK_3",   "type": "PICK",    "x": 3.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_3",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 4 ---
    {"name": "PICK_4",   "type": "PICK",    "x": 4.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_4",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 5 ---
    {"name": "PICK_5",   "type": "PICK",    "x": 5.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_5",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 6 ---
    {"name": "PICK_6",   "type": "PICK",    "x": 6.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_6",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 7 ---
    {"name": "PICK_7",   "type": "PICK",    "x": 7.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_7",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 8 ---
    {"name": "PICK_8",   "type": "PICK",    "x": 8.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_8",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 9 ---
    {"name": "PICK_9",   "type": "PICK",    "x": 9.0,  "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_9",  "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- CYCLE 10 ---
    {"name": "PICK_10",  "type": "PICK",    "x": 10.0, "y": 1.0,  "yaw": 0.0},
    {"name": "DEPOT_10", "type": "DEPOSIT", "x": 10.0, "y": 15.0, "yaw": 1.57},

    # --- RETOUR FINAL ---
    {"name": "FIN_MATCH", "type": "NONE",    "x": 0.0,  "y": 0.0,  "yaw": 0.0},
]

# ==============================================================================
# CLASSE PRINCIPALE : GESTIONNAIRE DE MISSION
# ==============================================================================
class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        # Communication Yahboom & Nav2
        self.servo_pub = self.create_publisher(Servo, '/servo', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Suivi d'état
        self.current_step = 0
        self.items_delivered = 0
        self.action_timer = None
        self.sub_step = 0

        self.get_logger().info("🔥 SPRINT 4MN : PRÊT POUR LE DÉPART")

        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("En attente de Nav2...")

        self.send_next_goal()

    def move_servo(self, servoid, angle):
        """Publie une commande de servo."""
        msg = Servo()
        msg.servoid, msg.angle = servoid, int(angle)
        self.servo_pub.publish(msg)

    # --------------------------------------------------------------------------
    # SEQUENCES MECANIQUES (ASYNCHRONES)
    # --------------------------------------------------------------------------
    def start_mechanical_action(self, action_type):
        """Déclenche la séquence bras/pelle sans bloquer le robot."""
        self.sub_step = 0
        interval = 0.8 # Vitesse entre étapes
        self.action_timer = self.create_timer(interval, self.tick_pick if action_type == "PICK" else self.tick_deposit)

    def tick_pick(self):
        """Ramassage sécurisé : Racleur d'abord, Pelle ensuite."""
        if self.sub_step == 0:
            self.move_servo(2, 180) # ÉTAPE 1 : Ouvrir racleur
        elif self.sub_step == 1:
            self.move_servo(1, 0)   # ÉTAPE 2 : Descendre pelle
        elif self.sub_step == 2:
            self.move_servo(2, 0)   # ÉTAPE 3 : Serrer le déchet
        elif self.sub_step == 3:
            self.move_servo(1, 90)  # ÉTAPE 4 : Remonter
        elif self.sub_step == 4:
            self.finish_action()
        self.sub_step += 1

    def tick_deposit(self):
        """Dépôt rapide."""
        if self.sub_step == 0:
            self.move_servo(1, 0)   # Descendre
        elif self.sub_step == 1:
            self.move_servo(2, 180) # Ouvrir
        elif self.sub_step == 2:
            self.move_servo(1, 90); self.move_servo(2, 0) # Remonter & Refermer
        elif self.sub_step == 3:
            self.items_delivered += 1
            self.get_logger().info(f"Dépôt {self.items_delivered}/10 effectué.")
            self.finish_action()
        self.sub_step += 1

    def finish_action(self):
        """Nettoie le timer et passe à l'étape suivante."""
        if self.action_timer:
            self.action_timer.cancel()
            self.action_timer = None
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
