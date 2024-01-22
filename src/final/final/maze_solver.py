'''
maze_solver.py

- description:
node to perform the actual task of maze solving

- usage:
write in the terminal where the package is sourced:
ros2 run final maze_solver

- inputs:
this node is subscribing video feed from sat view and bot view

- outputs:
this node publishes on topic "/cmd_vel", the required velocity -linear and angular- to move the robot

- author:
Jesús Parra Torrijos | @jesusparrat

- date:
20/01/2024
'''


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import time # importing time to know the time taken to solve the maze

from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
from .bot_pathplanning import bot_pathplanner
from .bot_motionplanning import bot_motionplanner

# creating a node class to perform the task of maze solving
class maze_solver(Node):
    def __init__(self): # defining the constructor of the class maze_solver
        super().__init__("maze_solving_node") # initializing the node with the name "maze_solving_node"
        self.inicio = time.time() # getting the current time

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # creating a publisher to publish the velocity of the robot
        self.videofeed_subscriber = self.create_subscription(Image, '/sat_camera/image_raw', self.get_video_feed_cb, 10) # creating a subscriber to get the video feed from the satellite camera
        self.bot_subscriber = self.create_subscription(Image, '/bot_camera/image_raw', self.process_data_bot, 10) # creating a subscriber to get the video feed from the robot camera

        self.timer = self.create_timer(0.2, self.maze_solving) # creating a timer to call the function maze_solving every 0.2 seconds
        self.bridge = CvBridge() # creating a CvBridge object to convert the images from ROS 2 to OpenCV
        self.vel_msg = Twist() # creating a Twist object to publish the velocity of the robot

        # creating objects for each stage of the robot navigation
        self.bot_localizer = bot_localizer()
        self.bot_mapper = bot_mapper()
        self.bot_pathplanner = bot_pathplanner()
        self.bot_motionplanner = bot_motionplanner()

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.bot_motionplanner.get_pose, 10) # creating a subscriber to get the pose of the robot

        self.sat_view = np.zeros((100,100)) # initializing the satellite view
        self.beginning_time = time.time() # getting the current time

    def get_video_feed_cb(self,data): # defining the callback function for the subscriber to get the video feed from the satellite camera
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8') # performing conversion
        self.sat_view = frame # updating the satellite view

    def process_data_bot(self, data):
      self.bot_view = self.bridge.imgmsg_to_cv2(data, 'bgr8') # performing conversion

    def maze_solving(self): # defining the function maze_solving
        frame_disp = self.sat_view.copy() # copying the satellite view to display the maze solving process

        # stage 1: localization: localizing robot at each iteration
        self.bot_localizer.localize_bot(self.sat_view, frame_disp)

        # stage 2: mapping: converting the image from sat view to a graph
        self.bot_mapper.graphify(self.bot_localizer.maze_og)

        # stage 3: path-planning to find paths to the goal
        start = self.bot_mapper.Graph.start
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze

        if not self.bot_pathplanner.dijsktra.shortestpath_found: # if the shortest path is not found using dijsktra's algorithm, then use a_star algorithm
            self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze, method = "dijsktra") # finding the shortest path using dijsktra's algorithm

        if not self.bot_pathplanner.astar.shortestpath_found: # if the shortest path is not found using a_star algorithm, then use dijsktra's algorithm
            self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze, method = "a_star") # finding the shortest path using a_star algorithm
            print("\nnodes visited by: [Dijsktra vs A*] = [{} vs {}]".format(self.bot_pathplanner.dijsktra.dijiktra_nodes_visited, self.bot_pathplanner.astar.astar_nodes_visited))

        # stage 4: motion-planning - reaching the maze exit by navigating the path previously computed
        bot_loc = self.bot_localizer.loc_car # getting the current location of the robot
        path = self.bot_pathplanner.path_to_goal # getting the path to the goal
        self.bot_motionplanner.nav_path(bot_loc, path, self.vel_msg, self.velocity_publisher) # navigating the path to the goal

        # displaying bot solving maze
        img_shortest_path = self.bot_pathplanner.img_shortest_path # getting the image of the shortest path
        self.bot_motionplanner.display_control_mechanism_in_action(bot_loc, path, img_shortest_path, self.bot_localizer, frame_disp) # displaying the control mechanism in action

        # view of the bot POV on left to frame display
        bot_view = cv2.resize(self.bot_view, (int(frame_disp.shape[0] / 2), int(frame_disp.shape[1] / 2)))
        frame_disp[0:bot_view.shape[0], 0:bot_view.shape[1]] = bot_view
        frame_disp[0:img_shortest_path.shape[0], frame_disp.shape[1] - img_shortest_path.shape[1]:frame_disp.shape[1]] = img_shortest_path

        self.final_time = time.time()
        print("\nTime taken to solve maze: {} seconds".format(self.final_time - self.beginning_time))

        cv2.imshow("FINAL", frame_disp) # displaying the final image
        cv2.waitKey(1) # waiting for a key press


def main(args = None): # defining the main function
    rclpy.init() # initializing the node
    node_obj = maze_solver() # creating an object of the class maze_solver
    rclpy.spin(node_obj) # running the node
    rclpy.shutdown() # shutting down the node

if __name__ == '__main__':
    main()