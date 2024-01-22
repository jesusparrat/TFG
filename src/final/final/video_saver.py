#!/usr/bin/env python3

'''
maze_solver.py

- description:
node to save video feed from the upper camera from the simulation which is going to be utilized for further image processing to solve the maze

- usage:
write in the terminal where the package is sourced:
ros2 run final video_recorder

- inputs:
this node is a subscriber of '/upper_camera/image_raw' topic which is 30 fps video from camera above the maze and image is of the size 1280x720

- outputs:
this node is just a subscriber - the output is not in terms of ROS topic but a video is going to be saved on the disk

- author:
Jesús Parra Torrijos | @jesusparrat

- date:
17/12/2024
'''

import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

# creating a node class to perform the video recording
class video_get(Node):
  def __init__(self): # defining the constructor of the class video_get
    super().__init__('video_subscriber') # node name

    self.subscriber = self.create_subscription(Image, '/upper_camera/image_raw', self.process_data, 10) # creating a subscriber to get the video feed from the satellite camera

    vid_path = os.path.join(os.getcwd(),"src/final/output.avi") # path to save the video

    self.out = cv2.VideoWriter(vid_path, cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1280, 720)) # creating a video writer object to save the video
    self.bridge = CvBridge() # converting ros images to OpenCV data
    self.window_name = "output"  # Añade esta línea para definir la variable

  def process_data(self, data):
    frame = self.bridge.imgmsg_to_cv2(data, 'bgr8') # converting ros images to OpenCV data
    self.out.write(frame) # saving the video

    cv2.imshow(self.window_name, frame) # showing the video feed
    cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL) # adding a resizable window to show the video feed
    cv2.resizeWindow(self.window_name, 1280, 720) # resizing the window

    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
  main()