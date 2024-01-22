'''
bot_localization.py

- description:
node to perform localization of robot using background subtraction from OpenCV.

- inputs:
1. extracted frame from video feed of satellite camera
2. frame to display the localized robot in the maze

- outputs:
1. self.car_loc - coordinates (x,y) of the localized car
2. self.maze_og - occupancy grid generated from the cropped maze

- author:
Jesús Parra Torrijos | @jesusparrat

- date:
09/01/2024
'''
import cv2
import numpy as np

from .utilities import ret_smallest_obj, ret_largest_obj

class bot_localizer():

    def __init__(self):

        # initialization of variables
        self.is_bg_extracted = False

        # output variables
        self.bg_model = [] # background model initialized as empty
        self.maze_og = [] # maze occupancy grid initialized as empty
        self.loc_car = 0 # location of the car initialized as 0

        # transformation, crop and rotated, variables
        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transform_arr = []

        # rotation variables
        self.orig_rot = 0
        self.rot_mat = 0

    @staticmethod
    def ret_rois_boundinghull(rois_mask,cnts): # creation of a function to return the hull of the maze
        maze_enclosure = np.zeros_like(rois_mask) # creation of a black image with the same size as the rois_mask
        if cnts: # if there are contours
            cnts_ = np.concatenate(cnts) # concatenate all the contours
            cnts_ = np.array(cnts_) # convert the contours to an array
            cv2.fillConvexPoly(maze_enclosure, cnts_, 255) # fill the maze_enclosure with the contours
        cnts_largest = cv2.findContours(maze_enclosure, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0] # find the largest contour
        hull = cv2.convexHull(cnts_largest[0]) # find the hull of the largest contour
        cv2.drawContours(maze_enclosure, [hull], 0, 255) # draw the hull
        return hull # return the hull

    def update_frameofrefrence_parameters(self,X,Y,W,H,rot_angle): # update the parameters of the frame of refrence
        self.orig_X = X; self.orig_Y = Y; self.orig_rows = H; self.orig_cols = W; self.orig_rot = rot_angle # update the parameters with 90 degree counterClockwise
        self.transform_arr = [X,Y,W,H] # update the transform array
        # rotation matrix
        self.rot_mat = np.array(
                                [
                                 [ np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))],
                                 [-np.sin(np.deg2rad(self.orig_rot)) , np.cos(np.deg2rad(self.orig_rot))]
                                ]
                               ) # update the rotation matrix
        self.rot_mat_rev = np.array(
                                [
                                 [ np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))],
                                 [-np.sin(np.deg2rad(-self.orig_rot)) , np.cos(np.deg2rad(-self.orig_rot))]
                                ]
                               ) # update the reverse rotation matrix

    @staticmethod # static method to connect objects
    def connect_objs(bin_img): # connect disconnected edges that are close enough
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)) # kernel to use in the morphological operation
        return(cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel)) # return the morphological operation

    def extract_bg(self,frame): # function to extract the background

        # a) find contours of all the regions of interest in frozen sat_view
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert the frame to gray
        edges = cv2.Canny(gray, 50, 150,None,3) # find the edges of the frame
        # [connect_objs] -> connect disconnected edges that are close enough
        edges = self.connect_objs(edges) # connect the edges
        cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0] # find the contours of the edges
        rois_mask = np.zeros((frame.shape[0],frame.shape[1]),dtype= np.uint8) # create a black image with the same size as the frame
        for idx,_ in enumerate(cnts): # for each contour
            cv2.drawContours(rois_mask, cnts, idx, 255,-1) # draw the contours in the black image

        # b) extract the background model by
        #       i)  removing the smallest object from the scene (bot)
        #       ii) filling the empty region with ground_replica
        min_cntr_idx = ret_smallest_obj(cnts) # find the smallest object
        rois_noCar_mask = rois_mask.copy() # copy the rois_mask

        if min_cntr_idx !=-1: # if the smallest object is found
            cv2.drawContours(rois_noCar_mask, cnts, min_cntr_idx, 0,-1) # draw the contours of the smallest object
            # drawing dilated car_mask
            car_mask = np.zeros_like(rois_mask) # create a black image with the same size as the rois_mask
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255,-1) # draw the contours of the smallest object with a thickness of -1
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255, 3) # draw the contours of the smallest object with a thickness of 3
            not_car_mask = cv2.bitwise_not(car_mask) # invert the car_mask
            frame_car_remvd = cv2.bitwise_and(frame, frame,mask = not_car_mask) # remove the car from the frame

            # generating ground replica from the frame to fill the empty space
            base_clr = frame_car_remvd[0][0] # get the color of the first pixel of the frame
            ground_replica = np.ones_like(frame)*base_clr # create a black image with the same size as the frame and fill it with the base color

            # generating bg_model
            self.bg_model = cv2.bitwise_and(ground_replica, ground_replica,mask = car_mask) # generate the bg_model by combining the frame_car_remvd and the ground_replica
            self.bg_model = cv2.bitwise_or(self.bg_model, frame_car_remvd) # generate the bg_model by combining the frame_car_remvd and the ground_replica

        # step 2: extracting the maze (frame of reference) with maze entry on top right
        # a) finding dimensions of hull enclosing largest contour
        hull = self.ret_rois_boundinghull(rois_mask,cnts) # find the hull of the maze
        [X,Y,W,H] = cv2.boundingRect(hull) # find the dimensions of the hull
        # b) cropping maze_mask from the image
        maze = rois_noCar_mask[Y:Y+H,X:X+W] # crop the maze from the rois_noCar_mask
        maze_occupancy_grid = cv2.bitwise_not(maze) # invert the maze
        self.maze_og = cv2.rotate(maze_occupancy_grid, cv2.ROTATE_90_COUNTERCLOCKWISE) # rotate the maze 90 degrees counterClockwise

        # storing crop and rotation parameters required to maintain frame of reference in the origin image
        self.update_frameofrefrence_parameters(X,Y,W,H,90) # update the parameters of the frame of reference

        cv2.imshow('regions of interest',rois_mask) # show the rois_mask
        cv2.imshow('maze with car removed',frame_car_remvd) # show the frame_car_remvd
        # cv2.imshow('1c. ground_replica',ground_replica) # show the ground_replica
        cv2.imshow('clean background model',self.bg_model) # show the bg_model
        cv2.imshow('maze occupancy grid',self.maze_og) # show the maze_og
        cv2.waitKey(0) # wait for a key to be pressed
        cv2.destroyAllWindows() # destroy all the windows when the key is pressed

    @staticmethod
    def get_centroid(cnt): # function to get the centroid of the car
        M = cv2.moments(cnt) # get the moments of the contour
        cx = int(M['m10']/M['m00']) # get the x coordinate of the centroid
        cy = int(M['m01']/M['m00']) # get the y coordinate of the centroid
        return (cy,cx) # return the centroid of the car (y,x)

    def get_car_loc(self,car_cnt,car_mask): # function to get the location of the car

        bot_cntr = self.get_centroid(car_cnt) # get the centroid of the car

        bot_cntr_arr =  np.array([bot_cntr[1],bot_cntr[0]]) # convert the centroid to an array to apply transforms
        # shift origin from sat_view to maze
        bot_cntr_translated = np.zeros_like(bot_cntr_arr) # create a black image with the same size as the bot_cntr_arr
        bot_cntr_translated[0] = bot_cntr_arr[0] - self.orig_X # shift the x coordinate of the centroid
        bot_cntr_translated[1] = bot_cntr_arr[1]-self.orig_Y # shift the y coordinate of the centroid

        # applying rotation transformation to bot_centroid to get bot location relative to maze
        bot_on_maze = (self.rot_mat @ bot_cntr_translated.T).T # apply the rotation matrix to the bot_cntr_translated

        # translating origin if necessary to get complete image
        rot_cols = self.orig_rows # get the number of columns of the rotated image
        rot_rows = self.orig_cols # get the number of rows of the rotated image
        bot_on_maze[0] = bot_on_maze[0] + (rot_cols * (bot_on_maze[0]<0) ) # translate the x coordinate of the bot_on_maze
        bot_on_maze[1] = bot_on_maze[1] + (rot_rows * (bot_on_maze[1]<0) ) # translate the y coordinate of the bot_on_maze

        # updating the placeholder for relative location of car
        self.loc_car = (int(bot_on_maze[0]),int(bot_on_maze[1]))

    def localize_bot(self,curr_frame,frame_disp): # function to localize the bot

        # background model extraction
        if not self.is_bg_extracted: # if the background model is not extracted
            self.extract_bg(curr_frame.copy()) # extract the background model
            self.is_bg_extracted = True # update the is_bg_extracted variable

        # foreground Detection
        change = cv2.absdiff(curr_frame, self.bg_model) # find the difference between the current frame and the background model
        change_gray = cv2.cvtColor(change, cv2.COLOR_BGR2GRAY) # convert the change to gray
        change_mask = cv2.threshold(change_gray, 15, 255, cv2.THRESH_BINARY)[1] # apply a threshold to the change
        car_mask, car_cnt = ret_largest_obj(change_mask) # find the largest object in the change_mask

        self.get_car_loc(car_cnt,car_mask) # get the location of the car

        # drawing bounding circle around detected car and displaying it
        center, radii = cv2.minEnclosingCircle(car_cnt) # find the center and the radius of the car
        car_circular_mask = cv2.circle(car_mask.copy(), (int(center[0]), int(center[1])), int(radii+(radii*0.4)), 255, 3) # draw a circle around the car
        car_circular_mask = cv2.bitwise_xor(car_circular_mask, car_mask) # apply a bitwise xor to the car_circular_mask and the car_mask
        frame_disp[car_mask>0]  = frame_disp[car_mask>0] + (0,64,0) # draw the car in the frame
        frame_disp[car_circular_mask>0]  = (0,0,255) # draw the circle around the car in the frame

        cv2.imshow("change_mask", change_mask) # show the change_mask
        cv2.imshow("detected car foreground", car_mask) # show the car_mask
        cv2.imshow("localized car in sat view", frame_disp) # show the frame_disp
        # cv2.waitKey(0) # wait for a key to be pressed
        # cv2.destroyAllWindows() # destroy all the windows when the key is pressed