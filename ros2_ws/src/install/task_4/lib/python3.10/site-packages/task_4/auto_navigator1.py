import sys
import os
import numpy as np
from queue import Queue # for A* alg

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from std_msgs.msg import Float32


# FOR A* ALGORITHM
class AStar():
    def __init__(self, in_tree):
        self.in_tree = in_tree
        self.q = Queue()
        self.dist = {name: np.Inf for name, node in in_tree.items()}
        self.h = {name: 0 for name, node in in_tree.items()}

        for name, node in in_tree.items():
            start = tuple(map(int, name.split(',')))
            end = tuple(map(int, self.in_tree['end'].split(',')))
            self.h[name] = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)

        self.via = {name: None for name, node in in_tree.items()}

    def __get_f_score(self, node):
    # start of my code
        return self.dist[node] + self.h[node]

    def solve(self, sn, en):
        self.dist[sn] = 0
        self.q.put(sn)

        while not self.q.empty():
            self.q.queue.sort(key=self.__get_f_score)
            u = self.q.get()

            if u == en:
                break
            for c, w in self.in_tree[u]:  # Access children and weight
                new_dist = self.dist[u] + w
                if new_dist < self.dist[c]:
                    self.dist[c] = new_dist
                    self.via[c] = u
                    self.q.put(c)
# end of my code
    
    def reconstruct_path(self,sn,en):
        path = []
        dist = 0
        u = en.name
# start of my code
        end_key = u
        start_key = sn.name

        dist = self.dist[end_key]
        u = end_key
        path = [u]

        while u != start_key:
            u = self.via[u]
            path.append(u)
        path.reverse()
# end of my code
        return path,dist
# END OF A * ALGORITHM       
 

class Navigation(Node):
    """! Navigation node class.
    This class should serve as a template to implement the path planning and
    path follower components to move the turtlebot from position A to B.
    """

    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        super().__init__(node_name)
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.start_time = 0.0

        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time',10) #DO NOT MODIFY

        # Node rate
        self.rate = self.create_rate(10)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        self.get_logger().info(
            'goal_pose: {:.4f}, {:.4f}'.format(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        self.ttbot_pose = data.pose
        self.get_logger().info(
            'ttbot_pose: {:.4f}, {:.4f}'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y))

    def a_star_path_planner(self, start_pose, end_pose):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        # START OF MY CODE
        start_x = int(start_pose.pose.position.x) # defines important position info
        start_y = int(start_pose.pose.position.y)
        end_x = int(end_pose.pose.position.x)
        end_y = int(end_pose.pose.position.y)

        start_node = f"{start_x},{start_y}"
        end_node = f"{end_x},{end_y}"

        
        astar = AStar(self.graph) # uses astar alg
        astar.solve(start_node, end_node)
        path_coords = astar.reconstruct_path(start_node, end_node)
        # END OF MY CODE
        
        path = Path()
        self.get_logger().info(
            'A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        self.start_time = self.get_clock().now().nanoseconds*1e-9 #Do not edit this line (required for autograder)
        
        # TODO done (1 iteration): IMPLEMENTATION OF THE A* ALGORITHM 
        
        # START OF MY CODE - changes type of position messages
        for coord in path_coords:
            pose = PoseStamped()
            pose.pose.position.x = int(coord.split(',')[0])
            pose.pose.position.y = int(coord.split(',')[1])
            path.poses.append(pose)
        # END OF MY CODE
        
      #  path.poses.append(start_pose)
      #  path.poses.append(end_pose)
        # Do not edit below (required for autograder)
        self.astarTime = Float32()
        self.astarTime.data = float(self.get_clock().now().nanoseconds*1e-9-self.start_time)
        self.calc_time_pub.publish(self.astarTime)
        
        return path

    def get_path_idx(self, path, vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position in the path pointing to the next goal pose to follow.
        """
        idx = 0
        # TODO done (1 iteration): IMPLEMENT A MECHANISM TO DECIDE WHICH POINT IN THE PATH TO FOLLOW idx <= len(path)
        
        # START OF MY CODE
        min_distance = 99999 
        idx = 0  


        for i in range(len(path.poses)):
            waypoint_x = path.poses[i].pose.position.x
            waypoint_y = path.poses[i].pose.position.y

            distance = ((vehicle_pose.pose.position.x - waypoint_x) ** 2 +
                        (vehicle_pose.pose.position.y - waypoint_y) ** 2) ** 0.5
            
            if distance < min_distance:
                min_distance = distance
                idx = i 

        return idx

    def path_follower(self, vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        speed = 0.0
        heading = 0.0
        # TODO: IMPLEMENT PATH FOLLOWER
        
        # START OF MY CODE
        
        robot_x = vehicle_pose.pose.position.x
        robot_y = vehicle_pose.pose.position.y

        goal_x = current_goal_pose.pose.position.x
        goal_y = current_goal_pose.pose.position.y

        diff_x = goal_x - robot_x
        diff_y = goal_y - robot_y

        heading = np.arctan2(diff_y, diff_x)

        speed = 0.2  # setting a fixed speed for now, coulf fix this later
        # END OF MY CODE
      
        return speed, heading

    def move_ttbot(self, speed, heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired speed.
        @param  heading   Desired yaw angle.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        # TODO: IMPLEMENT YOUR LOW-LEVEL CONTROLLER
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading

        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        while rclpy.ok():
            # Call the spin_once to handle callbacks
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks without blocking

            # 1. Create the path to follow
            path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            # 2. Loop through the path and move the robot
            idx = self.get_path_idx(path, self.ttbot_pose)
            current_goal = path.poses[idx]
            speed, heading = self.path_follower(self.ttbot_pose, current_goal)
            self.move_ttbot(speed, heading)

            self.rate.sleep()
            # Sleep for the rate to control loop timing


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
