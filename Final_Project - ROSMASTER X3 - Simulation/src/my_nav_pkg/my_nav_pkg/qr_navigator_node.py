import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import math
import numpy as np
import heapq
from itertools import permutations
import time
import os
import cv2

# -------------------------------
# A* Planner
# -------------------------------
class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.height, self.width = grid.shape

    def heuristic(self, a, b):
        return math.hypot(b[0]-a[0], b[1]-a[1])

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny, nx] == 0:
                    neighbors.append((nx, ny))
        return neighbors

    def plan(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, [start]))
        visited = set()
        while open_set:
            _, cost, node, path = heapq.heappop(open_set)
            if node in visited:
                continue
            visited.add(node)
            if node == goal:
                return path
            for neighbor in self.get_neighbors(node):
                if neighbor not in visited:
                    new_cost = cost + 1
                    heapq.heappush(open_set, (new_cost + self.heuristic(neighbor, goal), new_cost, neighbor, path + [neighbor]))
        return None

# -------------------------------
# QR Navigator avancé
# -------------------------------
class QRNavigator(Node):
    def __init__(self):
        super().__init__('qr_navigator_advanced')

        # Publishers / Clients
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Carte
        map_file = '/home/billy/trc_ws/src/maps/trc_arena.yaml'
        self.grid, self.origin, self.resolution = self.load_map(map_file)

        # Positions
        self.positions = [
            {'name':'init', 'x':0.255, 'y':0.0, 'yaw':-1.611},
            {'name':'Z1', 'x':0.892, 'y':-0.286, 'yaw':-1.823},
            {'name':'Z2', 'x':1.741, 'y':-1.134, 'yaw':-3.065},
            {'name':'Z3', 'x':0.911, 'y':0.350, 'yaw':-1.018},
            {'name':'Z4', 'x':0.918, 'y':0.282, 'yaw':1.989},
            {'name':'Z5', 'x':1.682, 'y':0.394, 'yaw':1.496},
            {'name':'Z6', 'x':2.206, 'y':0.146, 'yaw':-2.335},
            {'name':'Z7', 'x':1.371, 'y':1.657, 'yaw':-2.582},
            {'name':'Depot', 'x':0.5, 'y':1.5, 'yaw':0.0}
        ]
        self.t_scan = 5
        self.astar = AStarPlanner(self.grid)
        self.obstacle_detected = False

        # Calcul ordre optimal
        self.opt_order = self.compute_optimal_order()

        # Navigation complète
        self.navigate_all()

    # -------------------------------
    # Carte
    # -------------------------------
    def load_map(self, yaml_file):
        with open(yaml_file, 'r') as f:
            map_data = yaml.safe_load(f)
        img_file = map_data['image']
        resolution = map_data['resolution']
        origin = map_data['origin']
        img_path = os.path.join(os.path.dirname(yaml_file), img_file)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        _, binary_map = cv2.threshold(img, 250, 1, cv2.THRESH_BINARY_INV)
        grid = np.array(binary_map, dtype=int)
        return grid, origin, resolution

    # -------------------------------
    # Conversion monde -> grille
    # -------------------------------
    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return gx, gy

    # -------------------------------
    # Publier chemin A*
    # -------------------------------
    def publish_astar_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = node[0]*self.resolution + self.origin[0] + self.resolution/2
            pose.pose.position.y = node[1]*self.resolution + self.origin[1] + self.resolution/2
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    # -------------------------------
    # Distance
    # -------------------------------
    def euclidean_dist(self, a, b):
        return math.hypot(b['x']-a['x'], b['y']-a['y'])

    # -------------------------------
    # Ordre optimal (TSP simple)
    # -------------------------------
    def compute_optimal_order(self):
        qr_positions = [p for p in self.positions if 'Depot' not in p and 'init' not in p]
        best_order = None
        min_dist = float('inf')
        for perm in permutations(qr_positions):
            dist = self.euclidean_dist(self.positions[0], perm[0])
            for i in range(len(perm)-1):
                dist += self.euclidean_dist(perm[i], perm[i+1])
            for pos in perm:
                dist += self.euclidean_dist(pos, self.get_position('Depot'))
            dist += self.euclidean_dist(perm[-1], self.positions[0])
            if dist < min_dist:
                min_dist = dist
                best_order = perm
        # Ajouter dépôt après chaque zone
        order = [self.positions[0]]
        for p in best_order:
            order.append(p)
            order.append(self.get_position('Depot'))
        return order

    def get_position(self, name):
        for pos in self.positions:
            if pos['name'] == name:
                return pos
        return None

    # -------------------------------
    # Callback scan (obstacle)
    # -------------------------------
    def scan_callback(self, msg: LaserScan):
        min_dist = min(msg.ranges)
        self.obstacle_detected = min_dist < 0.3  # obstacle < 30cm

    # -------------------------------
    # Envoyer goal avec contrôle dynamique
    # -------------------------------
    def send_goal_dynamic(self, pos, max_speed=1.0):
        goal_msg = NavigateToPose.Goal()
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = pos['x']
        pose_msg.pose.position.y = pos['y']
        pose_msg.pose.orientation.z = math.sin(pos['yaw']/2.0)
        pose_msg.pose.orientation.w = math.cos(pos['yaw']/2.0)
        goal_msg.pose = pose_msg

        self.get_logger().info(f"Envoi vers {pos['name']}")
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejeté')
            return

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            # Ajuster vitesse en fonction d'obstacle
            twist = Twist()
            twist.linear.x = max_speed if not self.obstacle_detected else 0.2
            twist.angular.z = 0.0  # rotation rapide si mecanum géré par nav2
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        rclpy.spin_until_future_complete(self, result_future)

        # Pause ramassage
        if 'Depot' not in pos['name'] and 'init' not in pos['name']:
            self.get_logger().info(f"Ramassage à {pos['name']} : {self.t_scan}s")
            time.sleep(self.t_scan)

    # -------------------------------
    # Navigation complète
    # -------------------------------
    def navigate_all(self):
        for pos in self.opt_order[1:]:
            # Tracer chemin
            start_grid = self.world_to_grid(self.positions[0]['x'], self.positions[0]['y'])
            goal_grid = self.world_to_grid(pos['x'], pos['y'])
            path = self.astar.plan(start_grid, goal_grid)
            if path:
                self.publish_astar_path(path)

            # Vitesse selon phase
            speed = 1.0 if 'Depot' not in pos['name'] else 0.5
            self.send_goal_dynamic(pos, max_speed=speed)

        # Retour initial
        self.send_goal_dynamic(self.positions[0], max_speed=0.5)
        self.get_logger().info("Retour à la position initiale")

# -------------------------------
# Main
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = QRNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

