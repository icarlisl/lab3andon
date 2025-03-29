import sys
import os
import numpy as np
import heapq

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from std_msgs.msg import Float32
from math import sqrt, atan2

class Navigation(Node):
    def __init__(self, node_name='Navigation'):
        super().__init__(node_name)
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.start_time = 0.0
       
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time', 10)

        self.rate = self.create_rate(10)

    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        self.get_logger().info(
            'goal_pose: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
        self.get_logger().info(f"Received goal: x={self.goal_pose.pose.position.x}, y={self.goal_pose.pose.position.y}")

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        self.get_logger().info(
            'ttbot_pose: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))
        self.get_logger().info(f"Updated robot pose: x={self.ttbot_pose.pose.position.x}, y={self.ttbot_pose.pose.position.y}")

    def a_star_path_planner(self, start_pose, end_pose):
        def heuristic(a, b):
            return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        start = (start_pose.pose.position.x, start_pose.pose.position.y)
        goal = (end_pose.pose.position.x, end_pose.pose.position.y)

        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                break

            for dx, dy in [(-0.1, 0), (0.1, 0), (0, -0.1), (0, 0.1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                new_cost = cost_so_far[current] + heuristic(current, neighbor)
               
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current

        path = Path()
        self.get_logger().info(
            'A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        path.poses.append(start_pose)
        current = goal
        while current in came_from:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = current
            path.poses.insert(1, pose)
            current = came_from[current]
        path.poses.append(end_pose)
       
        self.get_logger().info(f"Generated Path: {[(p.pose.position.x, p.pose.position.y) for p in path.poses]}")

        self.astarTime = Float32()
        self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
        self.calc_time_pub.publish(self.astarTime)
        return path

    def get_path_idx(self, path, vehicle_pose):
        min_dist = float('inf')
        idx = 0
        for i, pose in enumerate(path.poses):
            dist = sqrt((pose.pose.position.x - vehicle_pose.pose.position.x) ** 2 +
                        (pose.pose.position.y - vehicle_pose.pose.position.y) ** 2)
            if dist < min_dist:
                min_dist = dist
                idx = i
        return min(idx + 1, len(path.poses) - 1)

    def path_follower(self, vehicle_pose, current_goal_pose):
        dx = current_goal_pose.pose.position.x - vehicle_pose.pose.position.x
        dy = current_goal_pose.pose.position.y - vehicle_pose.pose.position.y
        distance = sqrt(dx**2 + dy**2)

        heading = atan2(dy, dx)
        speed = min(0.5, distance)

        return speed, heading

    def move_ttbot(self, speed, heading):
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            idx = self.get_path_idx(path, self.ttbot_pose)
            current_goal = path.poses[idx]
            speed, heading = self.path_follower(self.ttbot_pose, current_goal)
            self.move_ttbot(speed, heading)
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    nav = Navigation(node_name='Navigation')

    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

