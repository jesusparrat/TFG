'''
bot_motionplanning.py

- description:
node to perform motion planning for helping the vehicle navigate to the desired destination

- inputs:
1. robot current location
2. found path to destination
3. velocity object for manipulating linear and angular component of robot
4. velocity publisher to publish the updated velocity object

- outputs:
1. speed              - speed with which the car travels at any given moment
2. angle              - amount of turning the car needs to do at any moment

- author:
Jesús Parra Torrijos | @jesusparrat

- date:
19/01/2024
'''
import cv2
import numpy as np
from math import pow, atan2, sqrt, degrees, asin

from numpy import interp
import pygame
import os
import time

pygame.mixer.init() # initializing pygame.mixer module otherwise pygame.mixer.music will have no attribute play
pygame.mixer.music.load(os.path.abspath('src/final/resource/pacman.mp3')) # loading the audio file of pacman eating when the car reaches the mini-goal

class bot_motionplanner():

    def __init__(self):
        # counter to move car forward for a few iterations
        self.count = 0

        # state variable: is initial point extracted?
        self.pt_i_taken = False
        # container: store initial car location
        self.init_loc = 0

        # state variable for the angle relation computed
        self.angle_relation_computed = False

        # container:  bot angle from image
        self.bot_angle = 0
        # container:  bot angle from simulation
        self.bot_angle_s = 0
        # container:  angle relation between image and simulation
        self.bot_angle_rel = 0

        # state variable for the maze exit  - is it not reached?
        self.goal_not_reached_flag = True

        # container:  mini-goal (x,y)
        self.goal_pose_x = 0
        self.goal_pose_y = 0

        # getting the current mini-goal iteration
        self.path_iter = 0

        # container:  previous iteration case: angle to turn or distance to goal or mini-goal
        self.prev_angle_to_turn = 0
        self.prev_distance_to_goal = 0
        self.prev_path_iter = 0

        # iterations elapsed since no there's no change or back-peddling
        self.angle_not_changed = 0
        self.dist_not_changed = 0
        self.goal_not_changed = 0
        self.goal_not_changed_long = 0
        self.back_peddling = 0

        # state variables: is it stuck on a wall? get ready to back-peddle
        self.trigger_back_peddling = False
        # [State Variables] Can't reach goal? Try next appropriate one
        self.trigger_nxtpt = False
        self.beginning = time.time() # getting the initial time

    @staticmethod
    def euler_from_quaternion(x, y, z, w): # def euler_from_quaternion in radians
        # its better to use this function rather than using the tf.transformations.euler_from_quaternion because it does not give the same results
        t0 = + 2.0 * (w * x + y * z)
        t1 = + 1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = + 2.0 * (w * y - z * x)
        t2 = + 1.0 if t2 > +1.0 else t2
        t2 = - 1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

    def get_pose(self,data): # defining a function to get the pose of the robot
        # we get the bot_turn_angle in simulation using same method as go_to_goal.py
        quaternions = data.pose.pose.orientation # getting the quaternions from the pose message
        (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w) # converting the quaternions to euler angles
        yaw_deg = degrees(yaw) # converting the yaw angle from radians to degrees

        # maintaining the consistency in the angle range
        if (yaw_deg > 0): # if the angle is positive
            self.bot_angle_s = yaw_deg # the angle is already in the range of [0,360]
        else: # if the angle is negative
            # -210º + 360º = 150º; -180º + 360º = 180º; -90º + 360º = 270º
            self.bot_angle_s = yaw_deg + 360 # the angle is converted to the range of [0,360]
        # we turn the bot location from the old coordinate system [-180,180] to the new coordinate system to [0,360]

    @staticmethod
    def back_to_origin(pt,transform_arr,rot_mat): # defining a function to get the point back to the origin

        st_col = transform_arr[0] # cols X
        st_row = transform_arr[1] # rows Y
        tot_cols = transform_arr[2] # total_cols / width W
        tot_rows = transform_arr[3] # total_rows / height H

        pt_array = np.array([pt[0], pt[1]]) # defining the point as an array

        # rotation matrix for normal XY convention around Z axis = [cos0 -sin0] but for image convention it is [ cos0 sin0]
        #                                                          [sin0  cos0]                                [-sin0 cos0]
        rot_center = (rot_mat @ pt_array.T).T # rotating the point around the origin

        # translating the origin if it's necessary to get the whole image
        rot_cols = tot_cols # tot_rows
        rot_rows = tot_rows # tot_cols
        rot_center[0] = rot_center[0] + (rot_cols * (rot_center[0] < 0)) + st_col # translating the origin in the X axis
        rot_center[1] = rot_center[1] + (rot_rows * (rot_center[1] < 0)) + st_row # translating the origin in the Y axis
        return rot_center # returning the point back to the origin

    # function to display the control mechanism in action
    def display_control_mechanism_in_action(self, bot_loc, path, img_shortest_path, bot_localizer, frame_disp):
        doing_pt = 0 # mini-goal completing
        done_pt = 0 # mini-goal completed

        path_i = self.path_iter # current mini-goal

        # circle to represent car current location
        img_shortest_path = cv2.circle(img_shortest_path, bot_loc, 3, (0, 0, 255)) # car current location

        if ((type(path) != int) and (path_i != (len(path) - 1))): # if the path is not empty and the car is not in the final goal
            curr_goal = path[path_i] # current mini-goal

            # mini-goal completed
            if path_i != 0: # if the car is not in the first mini-goal
                img_shortest_path = cv2.circle(img_shortest_path, path[path_i - 1], 3, (0, 255, 0), 2) # if the car is not in the first mini-goal, we draw a circle in the previous mini-goal
                done_pt = path[path_i-1] # setting the previous mini-goal as done_pt
            # mini-goal completing
            img_shortest_path = cv2.circle(img_shortest_path, curr_goal, 3, (0, 140, 255), 2) # drawing a circle in the current mini-goal
            doing_pt = curr_goal # setting the current mini-goal as doing_pt
        else: # if the car is in the final goal
            # only display the final goal completed
            img_shortest_path = cv2.circle(img_shortest_path, path[path_i], 10, (0, 255, 0)) # drawing a circle in the final goal
            done_pt = path[path_i] # setting the final goal as done_pt

        if doing_pt!=0: # if the car is not in doing a mini-goal
            doing_pt = self.back_to_origin(doing_pt, bot_localizer.transform_arr, bot_localizer.rot_mat_rev) # getting the doing_pt back to the origin
            frame_disp = cv2.circle(frame_disp, (int(doing_pt[0]), int(doing_pt[1])), 3, (0, 140, 255), 2) # drawing a circle in the doing_pt
            #loc_car_ = self.back_to_origin(loc_car, bot_localizer_obj.transform_arr, bot_localizer_obj.rot_mat_rev)
            #frame_disp = cv2.circle(frame_disp, (int(loc_car_[0]),int(loc_car_[1])), 3, (0,0,255))

        if done_pt!=0: # if te car is not done with a mini-goal
            done_pt = self.back_to_origin(done_pt, bot_localizer.transform_arr, bot_localizer.rot_mat_rev) # getting the done_pt back to the origin
            if ((type(path) != int) and (path_i != (len(path) - 1))): # if the path is not empty and the car is not in the final goal
                pass
                #frame_disp = cv2.circle(frame_disp, (int(done_pt[0]),int(done_pt[1])) , 3, (0,255,0),2)
            else: # if the car is in the final goal
                frame_disp = cv2.circle(frame_disp, (int(done_pt[0]), int(done_pt[1])), 10, (0, 255, 0)) # drawing a circle in the final goal

        st = "mini-goals total = {}, mini-goals done = {}".format(len(path), self.path_iter) # string to display the length of the path and the current mini-goal
        frame_disp = cv2.putText(frame_disp, st, (bot_localizer.orig_X - 10, bot_localizer.orig_Y - 30), 5, 0.79, 0) # displaying the string with the mini-goals total and done
        cv2.imshow("maze with shortest route + car location", img_shortest_path) # displaying the maze with the shortest route and the car location

    @staticmethod
    def angle_n_dist(pt_a, pt_b): # defining a function to get the angle and distance between two points
        # we have a problem with the angle convention between the image and the simulation
        #   Simulation     [ Image ]
        #
        #   Y                  ____X
        #    |                |
        #    |___             |
        #         X         Y
        # solution: subtract the first point in Y axis with the second point in Y axis
        error_x = pt_b[0] - pt_a[0] # error in X axis
        error_y = pt_a[1] - pt_b[1] # error in Y axis

        # calculating distance between two points
        distance = sqrt(pow((error_x), 2) + pow((error_y), 2)) # better with pow than with **

        # calculating angle between two points
        angle = atan2(error_y, error_x) # angle in radians
        angle_deg = degrees(angle) # convert angle to degrees

        if (angle_deg > 0): # if the angle is positive
            return (angle_deg), distance # the angle is already in the range of [0,360]
        else: # if the angle is negative
            return (angle_deg + 360), distance # the angle is converted to the range of [0,360]

    def check_gtg_status(self, angle_to_turn, distance_to_goal): # definingg a function to check the status of the go to oal

        # computing change in angle over the past iteration
        change_angle_to_turn = abs(angle_to_turn - self.prev_angle_to_turn)

        if( (abs(angle_to_turn) > 5) and (change_angle_to_turn < 0.4) and (not self.trigger_back_peddling)): # if the angle is large and the its not changing much and not already self.back_peddling then we increase the angle_not_changed
            self.angle_not_changed += 1 # increasing the angle_not_changed

            # set a significant time for the angle not changed then trigger self.back_peddling to move car in reverse
            if(self.angle_not_changed > 200): # if the angle_not_changed is greater than 200
                self.trigger_back_peddling = True # trigger self.back_peddling
        else: # if the angle is not large and the its changing much and not already self.back_peddling then we reset the angle_not_changed
            self.angle_not_changed = 0 # resetting the angle_not_changed
        print("[prev, change, not_changed_iter, self.trigger_back_peddling] = [{:.1f}, {:.1f}, {}, {}] ".format(self.prev_angle_to_turn, change_angle_to_turn, self.angle_not_changed, self.trigger_back_peddling)) # printing the previous angle to turn, change in angle to turn, angle not changed iterations and trigger self.back_peddling
        self.prev_angle_to_turn = angle_to_turn # setting the previous angle to turn as the current angle to turn

        # computing change in distance to goal over the past iteration and if the distance is large and the its not changing much and not already self.back_peddling then we increase the dist_not_changed
        change_dist = abs(distance_to_goal - self.prev_distance_to_goal) # computing change in distance over the past iteration

        if( (abs(distance_to_goal) > 5) and (change_dist<1.2) and (not self.trigger_back_peddling)): # if the distance is large and the its not changing much and not already self.back_peddling then we increase the dist_not_changed
            self.dist_not_changed += 1 # increasing the dist_not_changed
            # set a significant time for the angle not changed then trigger self.back_peddling to move car in reverse
            if(self.dist_not_changed>200): # if the dist_not_changed is greater than 200
                self.trigger_back_peddling = True # trigger self.back_peddling
        else: # if the distance is not large and the its changing much and not already self.back_peddling then we reset the dist_not_changed
            self.dist_not_changed = 0 # resetting the dist_not_changed
        print("[prev_d, change_d, not_changed_iter, self.trigger_back_peddling] = [{:.1f}, {:.1f}, {}, {}] ".format(self.prev_distance_to_goal, change_dist, self.dist_not_changed, self.trigger_back_peddling)) # printing the previous distance to goal, change in distance to goal, distance not changed iterations and trigger self.back_peddling
        self.prev_distance_to_goal = distance_to_goal # setting the previous distance to goal as the current distance to goal

        # computing change in mini-goal over the past iteration
        change_goal = self.prev_path_iter - self.path_iter # computing change in mini-goal over the past iteration
        if( (change_goal == 0) and (distance_to_goal < 30) ): # if mini-goal not changing and distance to goal is less than 30
            self.goal_not_changed += 1 # increasing the goal_not_changed
            # set a significant time for the angle not changed then trigger self.back_peddling to move car in reverse
            if(self.goal_not_changed > 500): # if the goal_not_changed is greater than 500
                self.trigger_nxtpt = True # trigger self.trigger_nxtpt
        elif(change_goal == 0): # if the mini-goal is not changing for a long time
            self.goal_not_changed_long += 1 # increasing the goal_not_changed_long
            if(self.goal_not_changed_long>1500): # and if the goal_not_changed_long is greater than 1500
                self.trigger_nxtpt = True # trigger self.trigger_nxtpt
        else: # if the mini-goal is changing
            self.goal_not_changed_long = 0 # resetting the goal_not_changed_long
            self.goal_not_changed = 0 # resetting the goal_not_changed
        print("[prev_g, change_g, not_changed_iter] = [{:.1f}, {:.1f}, {}] ".format(self.prev_path_iter, change_goal, self.goal_not_changed))
        self.prev_path_iter = self.path_iter

    @staticmethod
    def dist(pt_a, pt_b): # defining a function to get the distance between two points
        error_x= pt_b[0] - pt_a[0] # error in X axis
        error_y= pt_a[1] - pt_b[1] # error in Y axis
        return(sqrt(pow((error_x), 2) + pow((error_y), 2))) # better with pow than with **

    def get_suitable_nxtpt(self, car_loc, path): # defining a function to get the suitable next mini-goal
        extra_i = 1 # extra iterations to get the suitable next mini-goal
        test_goal = path[self.path_iter + extra_i]

        while(self.dist(car_loc, test_goal)<20):
            extra_i += 1
            test_goal = path[self.path_iter + extra_i]
        print("loading {} pt ".format(extra_i))
        self.path_iter = self.path_iter + extra_i - 1


    def go_to_goal(self, bot_loc, path, velocity, velocity_publisher): # defining a function to go to the goal
        # finding the distance and angle between current bot location and the current mini-goal
        angle_to_goal, distance_to_goal = self.angle_n_dist(bot_loc, (self.goal_pose_x, self.goal_pose_y))
        # computing the angle the bot needs to turn to align with the mini goal
        angle_to_turn = angle_to_goal - self.bot_angle
        # setting speed of bot proportional to its distance to the goal
        speed = interp(distance_to_goal, [0, 100], [0.2, 1.5]) # setting speed of bot proportional to its distance to the goal
        # setting steering angle of bot proportional to the amount of turn it is required to take
        angle = interp(angle_to_turn, [-360, 360], [-4, 4]) # setting the steering angle of the bot proportional to the amount of turn it is required to take

        print("angle to goal = {} angle_to_turn = {} angle[sim] {}".format(angle_to_goal, angle_to_turn, abs(angle))) # printing the angle to goal, angle to turn and the angle in the simulation
        print("distance to goal = ", distance_to_goal) # printing the distance to goal

        if self.goal_not_reached_flag: # if the goal is not reached
            self.check_gtg_status(angle_to_turn, distance_to_goal) # check the status of the go to goal

        # if car is far away, turn towards goal
        if (distance_to_goal >= 2): # if the distance to goal is greater than 2
            velocity.angular.z = angle # setting the angular velocity of the bot as the angle

        # in view of the limitation of differential drive, adjust speed of car with the amount of turn as the larger the turn, the slower the speed
        if abs(angle) < 0.4: # if the angle is less than 0.4
            velocity.linear.x = speed # setting the linear velocity of the bot as the speed
        elif((abs(angle) < 0.8)): # if the angle is less than 0.8
            velocity.linear.x = 0.02 # setting the linear velocity of the bot as 0.02
        else: # if the angle is greater than 0.8
            velocity.linear.x = 0.00001 # setting the linear velocity of the bot as 0.0

        if self.trigger_back_peddling: # if self.back_peddling is triggered
            print("###### back_peddling: ", self.back_peddling, " ######") # printing the back_peddling
            if self.back_peddling == 0: # and if the back_peddling is 0
                self.trigger_nxtpt = True # trigger self.trigger_nxtpt

            # make the car reverse by setting linear component negative
            velocity.linear.x = -0.16 # setting the linear velocity of the bot as -0.16
            velocity.angular.z = angle # setting the angular velocity of the bot as the angle
            self.back_peddling += 1 # increasing the back_peddling

            # stop the back_peddling after some time
            if self.back_peddling == 100: # if the back_peddling is 100
                self.trigger_back_peddling = False # trigger self.back_peddling
                self.back_peddling = 0 # resetting the back_peddling
                print("###### back_peddling DONE ######") # printing that the back_peddling is done

        # keep publishing the updated velocity until the final goal is not reached
        if (self.goal_not_reached_flag) or (distance_to_goal <= 1): # if the goal is not reached or the distance to goal is less than 1
            velocity_publisher.publish(velocity) # publishing the velocity

        # if car is within reasonable distance of mini-goal
        if ((distance_to_goal <= 8) or self.trigger_nxtpt): # if the distance to goal is less than 8 or self.trigger_nxtpt is triggered
            if self.trigger_nxtpt: # and if self.trigger_nxtpt is triggered
                if self.back_peddling: # and also if self.back_peddling is triggered
                    # we're stuck in a wall so start looking for appropriate next mini-goal
                    self.get_suitable_nxtpt(bot_loc,path) # getting the suitable next mini-goal
                self.trigger_nxtpt = False # trigger self.trigger_nxtpt

            velocity.linear.x = 0.0 # setting the linear velocity of the bot as 0.0
            velocity.angular.z = 0.0 # setting the angular velocity of the bot as 0.0

            if self.goal_not_reached_flag: # if the goal is not reached
                velocity_publisher.publish(velocity) # publishing the velocity

            # reached the final goal
            if self.path_iter == (len(path) - 1): # if the current mini-goal is the final goal
                if self.goal_not_reached_flag: # and if the goal is not reached
                    self.final_time = time.time() # getting the final time
                    print("-------------------------------------------------" + 
                          "\n\n\nTime taken to solve maze: {:.3f} seconds\n\n\n".format(self.final_time - self.beginning) +
                          "-------------------------------------------------")
 # printing the time taken to solve the maze
                    self.goal_not_reached_flag = False # setting the goal_not_reached_flag as False

                    # play the song when the goal is reached
                    pygame.mixer.music.load(os.path.abspath('src/final/resource/goal_reached.wav'))
                    pygame.mixer.music.play()

            # is the car still doing mini-goals?
            else: # if the current mini-goal is not the final goal
                # iterate over the next mini-goal
                self.path_iter += 1 # increasing the path_iter
                self.goal_pose_x = path[self.path_iter][0] # setting the goal_pose_x as the x coordinate of the current mini-goal
                self.goal_pose_y = path[self.path_iter][1] # setting the goal_pose_y as the y coordinate of the current mini-goal

                if pygame.mixer.music.get_busy() == False: # if the song is not playing
                    pygame.mixer.music.play() # play the song

    def nav_path(self, bot_loc, path, velocity, velocity_publisher): # defining a function to navigate the path
        # if valid path founds
        if (type(path) != int): # if the path is not empty
            # trying to reach first mini-goal
            if (self.path_iter == 0): # and if the current mini-goal is the first mini-goal
                self.goal_pose_x = path[self.path_iter][0] # setting the goal_pose_x as the x coordinate of the current mini-goal
                self.goal_pose_y = path[self.path_iter][1] # setting the goal_pose_y as the y coordinate of the current mini-goal

        if (self.count >20): # if the count is greater than 20
            if not self.angle_relation_computed: # and if the angle relation is not computed
                velocity.linear.x = 0.0 # stopping our car
                velocity_publisher.publish(velocity) # publishing the velocity

                self.bot_angle, _= self.angle_n_dist(self.init_loc, bot_loc) # extracting the car angle from the car initial location and the car final location after moving forward 50 iterations
                self.bot_angle_init = self.bot_angle # setting the car angle as the car angle initial

                self.bot_angle_rel = self.bot_angle_s - self.bot_angle # finding the relation coefficient between the car angle in the image and the car angle in the simulation
                self.angle_relation_computed = True # setting the angle relation as computed

            else: # if the angle relation is computed
                self.bot_angle = self.bot_angle_s - self.bot_angle_rel # finding the car angle from the car angle in the simulation and the relation coefficient between the car angle in the image and the car angle in the simulation

                print("\n\ncar angle = {} angle relation {} car angle from sim = {}".format(self.bot_angle, self.bot_angle_rel, self.bot_angle_s)) # printing the car angle, angle relation and car angle from simulation
                print("car angle_initial from the image = ", self.bot_angle_init) # printing the car angle initial from the image
                print("car location {}".format(bot_loc)) # printing the car location

                self.go_to_goal(bot_loc, path, velocity, velocity_publisher) # going to the goal


        else: # if the count is not greater than 20
            if not self.pt_i_taken: #and bot initial location not already taken
                self.init_loc = bot_loc # setting the bot initial location as the current bot location
                self.pt_i_taken = True # setting the bot initial location as taken

            # keep moving forward for 20 iterations
            velocity.linear.x = 1.0 # setting the linear velocity of the bot as 1.0
            velocity_publisher.publish(velocity) # publishing the velocity

            self.count += 1 # increasing the count