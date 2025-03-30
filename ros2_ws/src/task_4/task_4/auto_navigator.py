import sys
import os
import numpy as np
from queue import PriorityQueue # for A* alg
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from std_msgs.msg import Float32
from PIL import Image
import yaml
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
from copy import copy
from math import atan2, sqrt, pi # for deepseek code
from transforms3d.euler import euler_from_quaternion ## for deepseek code

class Map():
    def __init__(self, map):
        self.package_path = get_package_share_directory('task_4')
        self.map_path = os.path.join(self.package_path, 'maps', map + '.yaml')
        self.map_im, self.map_df, self.limits = self.__open_map(map)
        self.image_array = self.__get_obstacle_map()

    def __open_map(self, map):
        # Open the YAML file which contains the map name and other
        # configuration parameters
        f = open(map + '.yaml', 'r')
        map_df = pd.json_normalize(yaml.safe_load(f))
        # Open the map image
        map = map_df.image[0]
        im = Image.open(map)
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        # Get the limits of the map. This will help to display the map with the correct amount of data
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] - im.size[1] * map_df.resolution[0]
        
        return im, map_df, [xmin, xmax, ymin, ymax]

    def __get_obstacle_map(self,map_im, map_df):
        img_array = np.reshape(list(self.map_im.getdata()),(self.map_im.size[1],self.map_im.size[0]))
        up_thresh = self.map_df.occupied_thresh[0]*255
        low_thresh = self.map_df.free_thresh[0]*255

        for j in range(self.map_im.size[0]):
            for i in range(self.map_im.size[1]):
                if img_array[i,j] > up_thresh:
                    img_array[i,j] = 255
                else:
                    img_array[i,j] = 0
        return img_array

class MapProcessor():
    def __init__(self,name):
        self.map = Map(name)
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        self.map_graph = Tree(name)

    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and
            (i < map_array.shape[0]) and
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value

    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)

    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.map.image_array[i][j] == 0:
                    self.__inflate_obstacle(kernel,self.inf_map_img_array,i,j,absolute)
        r = np.max(self.inf_map_img_array)-np.min(self.inf_map_img_array)
        if r == 0:
            r = 1
        self.inf_map_img_array = (self.inf_map_img_array - np.min(self.inf_map_img_array))/r
        
        # four lines for debugging
        print("Original obstacle count:", np.sum(self.map.image_array == 0))
        print("Inflated obstacle count:", np.sum(self.inf_map_img_array > 0.5))
        plt.imshow(self.inf_map_img_array)
        plt.show()

    def get_graph_from_map(self):
        # Create the nodes that will be part of the graph, considering only valid nodes or the free space
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        # Connect the nodes through edges
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    if (i > 0):
                        neighbor = '%d,%d'%(i-1,j)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i-1][j] == 0:
                                # add an edge up
                                child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                    if (i < (self.map.image_array.shape[0] - 1)):
                        neighbor = '%d,%d'%(i+1,j)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i+1][j] == 0:
                            # add an edge down
                                child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                    if (j > 0):
                        neighbor = '%d,%d'%(i,j-1)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i][j-1] == 0:
                            # add an edge to the left
                                child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                    if (j < (self.map.image_array.shape[1] - 1)):
                        neighbor = '%d,%d'%(i,j+1)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i][j+1] == 0:
                            # add an edge to the right
                                child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                    if ((i > 0) and (j > 0)):
                        neighbor = '%d,%d'%(i-1,j-1)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i-1][j-1] == 0:
                            # add an edge up-left
                                child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                    if ((i > 0) and (j < (self.map.image_array.shape[1] - 1))):
                        neighbor = '%d,%d'%(i+1,j+1)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i-1][j+1] == 0:
                            # add an edge up-right
                                child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j > 0)):
                        neighbor = '%d,%d'%(i+1,j-1)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i+1][j-1] == 0:
                            # add an edge down-left
                                child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1))):
                        neighbor = '%d,%d'%(i+1,j+1)
                        if neighbor in self.map_graph.g:
                            if self.inf_map_img_array[i+1][j+1] == 0:
                            # add an edge down-right
                                child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                                self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])

    def gaussian_kernel(self, size, sigma=1):
        size = int(size) // 2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g =  np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        r = np.max(g)-np.min(g)
        sm = (g - np.min(g))*1/r
        return sm

    def rect_kernel(self, size, value):
        m = np.ones(shape=(size,size))
        return m

    def draw_path(self,path):
        path_tuple_list = []
        path_array = copy(self.inf_map_img_array)
        for idx in path:
            tup = tuple(map(int, idx.split(',')))
            path_tuple_list.append(tup)
            path_array[tup] = 0.5
        return path_array
        
        
                            
mp = MapProcessor('map')

kr = mp.rect_kernel(5,1)
#kr = mp.rect_kernel(1,1)
mp.inflate_map(kr,True)

mp.get_graph_from_map()

fig, ax = plt.subplots(dpi=100)
plt.imshow(mp.inf_map_img_array)
plt.colorbar()
plt.show()


# FOR A* ALGORITHM
class AStar():
    def __init__(self, in_tree, end_node):
        self.in_tree = in_tree
        self.end_node = end_node ## deepseek
        self.q = PriorityQueue()
        self.dist = {name: np.Inf for name, node in in_tree.items()}
        self.h = {name: 0 for name, node in in_tree.items()}

        end = tuple(map(int, self.end_node.split(',')))
        for name, node in in_tree.items():
            start = tuple(map(int, name.split(',')))
            # end = tuple(map(int, self.in_tree['end'].split(',')))
            self.h[name] = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)

        self.via = {name: None for name, node in in_tree.items()}

    def __get_f_score(self, node):
    # start of my code
        return self.dist[node] + self.h[node]

    def solve(self, sn, en):
        self.dist[sn] = 0
        self.q.put((0, sn))

        while not self.q.empty():
            current = self.q.get()[1]  
            if current == en:
                break
            
            current_node = self.in_tree[current]
            for child, weight in current_node.children.items():  # Assuming children dict exists
                new_cost = self.dist[current] + weight
                if new_cost < self.dist[child.name]:
                    self.dist[child.name] = new_cost
                    self.via[child.name] = current
                    priority = new_cost + self.h[child.name]
                    self.q.put((priority, child.name))
            
            #self.q.queue.sort(key=self.__get_f_score)
            #u = self.q.get()

            #if u == en:
            #    break
            #for c, w in self.in_tree[u]:  # Access children and weight
            #    new_dist = self.dist[u] + w
            #    if new_dist < self.dist[c]:
            #        self.dist[c] = new_dist
            #        self.via[c] = u
            #        self.q.put(c)
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
        
        # load map
        self.map = Map("map")
        # create obstacle map
        self.map_processor = MapProcessor("map")
        self.map_processor.inflate_map(self.map_processor.rect_kernel(5, 1), True)
        self.map_processor.get_graph_from_map()
        
   
        
    # converts coords version 2?
    def world_to_grid(self, x, y):
        res = self.map.map_df.resolution[0]
        return (
            int((x - self.map.limits[0]) / res),
            int((self.map.limits[3]) - y / res)
        )

    def grid_to_world(self, i, j):
        res = self.map.map_df.resolution[0]
        return (
            i * res + self.map.limits[0],
            self.map.limits[3] - (j * res)
        )    


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
        start_x = self.world_to_grid(start_pose.pose.position.x) # defines important position info
        start_y = self.world_to_grid(start_pose.pose.position.y)
        end_x = self.world_to_grid(end_pose.pose.position.x)
        end_y = self.world_to_grid(end_pose.pose.position.y)

        start_node = f"{start_x},{start_y}"
        end_node = f"{end_x},{end_y}"
        
        # debug ifs
        if start_node not in self.map_processor.map_graph.g:
            self.get_logger().error(f"Start node {start_node} not in graph!")
            return Path()

        if end_node not in self.map_processor.map_graph.g:
            self.get_logger().error(f"End node {end_node} not in graph!")
            return Path()

        
        astar = AStar(self.map_processor.map_graph.g, end_node) # uses astar alg
        astar.solve(start_node, end_node)
        path_coords = astar.reconstruct_path(self.map_processor.map_graph.g[f"{start_x},{start_y}"], self.map_processor.map_graph.g[f"{end_x},{end_y}"])
        # END OF MY CODE
        
        path = Path()
        
        # two debug lines
        self.get_logger().info(f"Start node: {start_node}, End node: {end_node}")
        self.get_logger().info(f"Path coords: {path_coords[0][:5]}")  # First 5 points
        
        if not pathcoords[0]:
            self.get_logger().error("No path found")
            return path # debugging
            
        if len(path_coords[0]) == 0:
            return path
        
        self.get_logger().info(
            'A* planner.\n> start: {},\n> end: {}'.format(start_pose.pose.position, end_pose.pose.position))
        self.start_time = self.get_clock().now().nanoseconds*1e-9 #Do not edit this line (required for autograder)
        
        # TODO done (1 iteration): IMPLEMENTATION OF THE A* ALGORITHM 
        
        # START OF MY CODE - changes type of position messages
        for coord in path_coords[0]:
            i, j = map(int, coord.split(','))
            x, y = self.grid_to_world(i, j)
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
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
        
        
        ## Deepseek code
        # Get current orientation
        current_ori = [
            vehicle_pose.pose.orientation.x,
            vehicle_pose.pose.orientation.y,
            vehicle_pose.pose.orientation.z,
            vehicle_pose.pose.orientation.w
        ]
        current_yaw = euler_from_quaternion(current_ori)[2]
  
        # Calculate angle difference
        target_yaw = np.arctan2(diff_y, diff_x)
        angle_diff = (target_yaw - current_yaw + np.pi) % (2*np.pi) - np.pi
        angular_speed = 1.0 * angle_diff  # P-controller

        # Add distance check
        distance = np.hypot(diff_x, diff_y)
        if distance < 0.1:  # Stop when close
            return 0.0, 0.0

        speed = min(0.2, distance*0.5)  # Adjust speed based on distance
       
       ## end of deepseek code
        
        
        
        
        
       

        heading = np.arctan2(diff_y, diff_x)

       # speed = 0.2  # setting a fixed speed for now, coulf fix this later
        # END OF MY CODE
      
        return speed, angular_speed

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
