'''
bot_mapping.py

- description:
node to perform mapping to convert maze view into a traversable graph

- inputs:
1. occupancy grid from localization stage

- outputs:
1. self.Graph.graph       - generated graph from provided maze occupancy grid
2. self.maze              - image displaying only pathways in maze

- author:
Jesús Parra Torrijos | @jesusparrat

- date:
12/01/2024
'''
import cv2
import numpy as np

draw_interest_points = True # flag to draw interest points on maze
debug_mapping = False # flag to debug mapping

# creating a graph class to store the interest points and their connected paths
class Graph():

    def __init__(self):
        # dictionary to store graph
        self.graph = {}

        self.start = 0 # Start point of maze
        self.end = 0  # End point of maze

    # creating a function to add new vertex to graph
    # if neighbor == None       then add a vertex
    #      if not None          then add connection
    def add_vertex(self, vertex, neighbor = None, case = None, cost = None):

        # if neighbor is present    then add a connection
        if vertex in self.graph.keys(): # if vertex is already present in graph
            self.graph[vertex][neighbor] = {} # adding neighbor to vertex
            self.graph[vertex][neighbor]["case"] = case # adding case to vertex
            self.graph[vertex][neighbor]["cost"] = cost # adding cost to vertex
        else:
            self.graph[vertex] = {} # adding vertex to graph
            self.graph[vertex]["case"] = case # adding case to vertex

    # function to display the complete graph
    def display_graph(self):
        for key, value in self.graph.items(): # looping over each vertex
            print("key {} has value of {} ".format(key, value)) # printing vertex and its neighbors

# bot_mapper class for performing  the mapping of robot navigation
class bot_mapper():

    def __init__(self):

        # state variables
        self.graphified = False # flag to check if graph is extracted from maze or not

        # cropping control for removing maze boundary in pixels
        self.crp_amt = 5

        # creating a graph object for storing the maze
        self.Graph = Graph()

        # state variables to define the connection status of each vertex
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

        # maze for displaying connection between nodes
        self.maze_connect = []

        # maze input
        self.maze = 0

    # display connection between nodes with a colored line
    def display_connected_nodes(self, curr_node, neighbor_node, case = "unknown", color = (0, 0, 255)): # default color is red
        curr_pixel = (curr_node[1], curr_node[0]) # converting curr_node to pixel coordinates
        neighbor_pixel = (neighbor_node[1], neighbor_node[0]) # converting neighbor_node to pixel coordinates

        print("--- CONNECTED >> {} << ".format(case)) # printing case

        self.maze_connect = cv2.line(self.maze_connect, curr_pixel, neighbor_pixel, color, 1) # drawing line between nodes
        cv2.imshow("nodes connected", self.maze_connect) # displaying the connection

        if debug_mapping: # if debug flag is true then wait for user input
            cv2.waitKey(0) # wait for user input
            self.maze_connect = cv2.line(self.maze_connect, curr_pixel, neighbor_pixel, (255, 255, 255), 1) # drawing line between nodes

    # connect curr_node to its neighbors in immediate regions
    def connect_neighbors(self, maze, node_row, node_col, case, step_l = 1, step_up = 0, total_connected = 0):
        curr_node = (node_row, node_col) # current node coordinates

        # checking if there's a path around the node
        if (maze[node_row - step_up][node_col - step_l] > 0):
            # if there is a path    then look for the neighbor node to connect
            neighbor_node = (node_row - step_up, node_col - step_l) # neighbor node coordinates

            # but if potential_neighbor_node is actually a key in graph
            if neighbor_node in self.Graph.graph.keys():
                neighbor_case = self.Graph.graph[neighbor_node]["case"] # then get its case
                cost = max(abs(step_l), abs(step_up)) # calculate cost of connection
                total_connected += 1 # increment total connected nodes

                self.Graph.add_vertex(curr_node, neighbor_node, neighbor_case, cost) # add connection to graph of current node
                self.Graph.add_vertex(neighbor_node, curr_node, case, cost) # add connection to graph of neighbor node
                print("\nconnected {} to {} with the case [step_l, step_up] = [{}, {}] and the cost is: {}".format(curr_node, neighbor_node, step_l, step_up, cost)) # printing connection details

                # if the vertex is not connected    then neighbor cycle through next possible routes to connect
                if not self.connected_left:
                    self.display_connected_nodes(curr_node, neighbor_node, "LEFT", (0, 0, 255)) # display connection between nodes
                    # the vertex has connected to its left neighbor
                    self.connected_left = True # set connected_left to true
                    # then checking up-left route now
                    step_l = 1 # set step left to 1
                    step_up = 1 # set step up to 1

                    self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, total_connected) # connect neighbors in top-left region

                if not self.connected_upleft: # if not connected to up-left neighbor
                    self.display_connected_nodes(curr_node, neighbor_node, "UPLEFT", (0, 128, 255)) # display connection between nodes
                    # the vertex has connected to its up-left neighbor
                    self.connected_upleft = True # set connected_up-left to true
                    # then checking top route now
                    step_l  = 0 # set step left to 0
                    step_up = 1 # set step up to 1

                    self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, total_connected) # connect neighbors in top region

                if not self.connected_up: # if not connected to up neighbor
                    self.display_connected_nodes(curr_node, neighbor_node, "UP",(0,255,0)) # display connection between nodes
                    # the vertex has connected to its up neighbor
                    self.connected_up = True # set connected_up to true
                    # then checking top-right route now
                    step_l  = -1 # set step left to -1 because we are looking right
                    step_up = 1 # set step up to 1 because we are looking up

                    self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, total_connected) # connect neighbors in top-right region

                if not self.connected_upright: # if not connected to up-right neighbor
                    self.display_connected_nodes(curr_node, neighbor_node, "UPRIGHT", (255, 0, 0)) # display connection between nodes
                    # the vertex has connected to its up-right neighbor
                    self.connected_upright = True # set connected_up-right to true

            # still searching for the node to connect in a respective direction
            if not self.connected_upright: # if it's not connected to up-right neighbor

                if not self.connected_left: # and if it's not connected to left neighbor also
                    # look a little more to the left
                    step_l += 1 # step left (increment)
                elif not self.connected_upleft: # and if it's not connected to up-left neighbor also
                    # look a little more diagnolly upleft
                    step_l += 1 # step left (increment)
                    step_up += 1 # step up (increment)
                elif not self.connected_up: # and if it's not connected to up neighbor also
                    # look a little more up
                    step_up += 1 # step up (increment)
                elif not self.connected_upright: # and if it's not connected to up-right neighbor also
                    # look a little more upright
                    step_l -=1 # step left (decrease)
                    step_up += 1 # step up (increment)

                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, total_connected) # connect neighbors in respective region
        else:
            # if there is no path in the direction we are looking, cycle to next direction
            if not self.connected_left: # if not connected to left neighbor
                # there should be a wall on left so start looking up left now
                self.connected_left = True # set connected_left to true
                # looking upleft now
                step_l = 1 # set step left to 1
                step_up = 1 # set step up to 1

                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, total_connected) # connect neighbors in top-left region

            elif not self.connected_upleft: # if not connected to up-left neighbor
                # there should be a wall up left so start looking up
                self.connected_upleft = True # set connected_up-left to true
                # looking up now
                step_l = 0 # set step left to 0
                step_up = 1 # set step up to 1

                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, total_connected) # connect neighbors in top region

            elif not self.connected_up: # if not connected to up neighbor
                # there should be a wall above so start looking up-right
                self.connected_up = True # set connected_up to true
                # looking up-right now
                step_l = -1 # set step left to -1 because we are looking right
                step_up = 1 # set step up to 1 because we are looking up

                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, total_connected) # connect neighbors in top-right region

            elif not self.connected_upright:
                # there should be a wall above so start looking up-right
                self.connected_upright = True # set connected_up-right to true
                step_l = 0 # set step left to 0 because we are looking right
                step_up = 0 # set step up to 0 because we are looking up
                return # return from function

    # function to draw a triangle around a point
    @staticmethod
    def triangle(image, ctr_pt, radius, colour = (0, 255, 255)): # default color is yellow
        # polygon corner points coordinates
        pts = np.array( [ [ctr_pt[0]         , ctr_pt[1] - radius],
                          [ctr_pt[0] - radius, ctr_pt[1] + radius],
                          [ctr_pt[0] + radius, ctr_pt[1] + radius]
                        ]
                        , np.int32
                      ) # defining polygon corner points

        pts = pts.reshape((-1, 1, 2)) # reshaping points to required format
        image = cv2.polylines(image, [pts], True, colour, 2) # drawing polygon on image
        return image # returning image with polygon drawn on it

    # function to get surrounding pixels intensities for any point
    @staticmethod
    def get_surround_pixel_intensities(maze, curr_row, curr_col):
        # binary thrsholding and setting + values t 1 and - values to 0
        maze = cv2.threshold(maze, 1, 1, cv2.THRESH_BINARY)[1] # threshold image to binary
        rows = maze.shape[0] # number of rows in maze
        cols = maze.shape[1] # number of columns in maze

        # state variables, if the point is at a boundary condition
        top_row = False
        bottom_row = False
        left_col = False
        right_col = False

        # checking if there's a boundary condition
        if (curr_row == 0): # if the point is at top row
            # if current row is the top row     then the row above is not accesible
            top_row = True # set top_row to true

        if (curr_row == (rows - 1)): # if the point is at bottom row
            # if current row is the bottom row     then the row below is not accesible
            bottom_row = True # set bottom_row to true

        if (curr_col == 0): # if the point is at left column
            # if current col is the left col     then the col to the left is not accesible
            left_col = True # set left_col to true

        if (curr_col == (cols - 1)): # if the point is at right column
            # if current col is the right col     then the col to the right is not accesible
            right_col = True # set right_col to true

        # extracting the surround pixel intensities and addressing the boundary conditions (if there are any)
        if (top_row or left_col): # if the point is at top row or left column
            top_left = 0 # set top_left to 0
        else: # if the point is not at top row or left column
            top_left = maze[curr_row - 1][curr_col - 1] # set top_left to the pixel intensity of top-left pixel

        if(top_row or right_col): # if the point is at top row or right column
            top_right = 0 # set top_right to 0
        else: # if the point is not at top row or right column
            top_right = maze[curr_row - 1][curr_col + 1] # set top_right to the pixel intensity of top-right pixel

        if(bottom_row or left_col): # if the point is at bottom row or left column
            bottom_left = 0 # set bottom_left to 0
        else: # if the point is not at bottom row or left column
            bottom_left = maze[curr_row + 1][curr_col - 1] # set bottom_left to the pixel intensity of bottom-left pixel

        if(bottom_row or right_col): # if the point is at bottom row or right column
            bottom_right = 0 # set bottom_right to 0
        else: # if the point is not at bottom row or right column
            bottom_right = maze[curr_row + 1][curr_col + 1] # set bottom_right to the pixel intensity of bottom-right pixel

        # if the point we are at is anywhere on the top row   then its top pixel is definitely not accesible
        if (top_row): # if the point is at top row
            top = 0 # set top to 0
        else: # if the point is not at top row
            top = maze[curr_row - 1][curr_col] # set top to the pixel intensity of top pixel

        if (right_col): # if the point is at right column
            right = 0 # set right to 0
        else: # if the point is not at right column
            right = maze[curr_row][curr_col + 1] # set right to the pixel intensity of right pixel

        if (bottom_row): # if the point is at bottom row
            bottom = 0 # set bottom to 0
        else: # if the point is not at bottom row
            bottom = maze[curr_row + 1][curr_col] # set bottom to the pixel intensity of bottom pixel

        if (left_col): # if the point is at left column
            left = 0 # set left to 0
        else: # if the point is not at left column
            left = maze[curr_row][curr_col - 1] # set left to the pixel intensity of left pixel

        no_of_pathways = ( top_left     + top         + top_right     +
                           left         + 0           + right         +
                           bottom_left  + bottom      + bottom_right    ) # calculating number of pathways around the point

        if no_of_pathways > 2: # if there are more than 2 pathways around the point
            print("[top_left, top, top_right, left, right, bottom_left, bottom, bottom_right] \n [", str(top_left),", ", str(top),", ", str(top_right)," ,\n  ", str(left),", ","-",", ", str(right)," ,\n  ", str(bottom_left),", ", str(bottom),", ", str(bottom_right)," ]")
            print("\nno_of_pathways [row, col] = [", curr_row,", ", curr_col,"] ", no_of_pathways)

        return top_left, top, top_right, right, bottom_right, bottom, bottom_left, left, no_of_pathways

    # reset state parameters of each vertex connection
    def reset_connct_paramtrs(self):
        # resetting member variables to false initially when looking for nodes to connect
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

    def one_pass(self, maze): # function to find interest points in maze
        # remove previously found nodes
        self.Graph.graph.clear()

        # initializing maze_connect with the maze
        self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR) # converting maze to colored
        cv2.namedWindow("nodes connected", cv2.WINDOW_FREERATIO) # creating a window to display connected nodes

        # initialize counts of interesting points
        turns = 0 # number of turns
        junc_3 = 0 # number of 3-junctions
        junc_4 = 0 # number of 4-junctions

        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR) # converting the maze to colored for identifying discovered interest points

        # creating a window to display the detected interest points
        cv2.namedWindow("maze with the interest points", cv2.WINDOW_FREERATIO)
        rows = maze.shape[0] # number of rows in maze
        cols = maze.shape[1] # number of columns in maze

        # looping over each pixel from left to right and bottom to top
        for row in range(rows): # looping over each row
            for col in range(cols): # looping over each column

                if (maze[row][col] == 255): # if the pixel is white
                    if debug_mapping: # if debug flag is true then wait for user input
                        self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR) # re-initializing the maze_connect with the colored maze

                    # probable interest point   then look to find the surrounding pixel intensities
                    top_left, top, top_right, right, bottom_right, bottom, bottom_left, left, paths = self.get_surround_pixel_intensities(maze.copy(), row, col)

                    if ((row == 0) or (row == (rows - 1)) or (col == 0) or (col == (cols - 1)) ): # if the pixel is at the boundary
                        if (row == 0): # and if the pixel is at top row
                            # start the maze
                            maze_bgr[row][col] = (0, 128, 255) # yellow color
                            cv2.imshow("maze with the interest points", maze_bgr)
                            # adding found vertex to graph and maze entry to graph-start
                            self.Graph.add_vertex((row, col), case = "_start_") # adding vertex to graph
                            self.Graph.start = (row, col) # setting start point of maze

                        else: # if the pixel is at bottom row
                            # end the maze exit
                            maze_bgr[row][col] = (0, 255, 0) # green color
                            cv2.imshow("maze with the interest points", maze_bgr) # displaying the maze with interest points
                            # adding found vertex to graph and maze entry to graph-end
                            self.Graph.add_vertex((row, col), case = "_end_") # adding vertex to graph
                            self.Graph.end = (row, col) # setting end point of maze
                            # connecting vertex to its neighbor (if there any)
                            self.reset_connct_paramtrs() # reset state parameters of each vertex connection
                            self.connect_neighbors(maze, row, col, "_end_") # connect neighbors of end point

                    # checking if it's a dead end
                    elif (paths == 1): # if there is only one pathway around the point
                        crop = maze[row - 1:row + 2, col - 1:col + 2] # cropping the region around the point
                        print("******* dead end *******", crop) # printing the cropped region
                        maze_bgr[row][col] = (0, 0, 255) # red color

                        if draw_interest_points: # if draw interest points flag is true
                            maze_bgr= cv2.circle(maze_bgr, (col, row), 10, (0, 0, 255), 2) # draw a circle around the point
                        cv2.imshow("maze with the interest points", maze_bgr) # displaying the maze with interest points
                        # adding found vertex to graph as a dead end
                        self.Graph.add_vertex((row, col), case = "_dead-end_")
                        # connecting vertex to its neighbor (if there's any)
                        self.reset_connct_paramtrs() # reset state parameters of each vertex connection
                        self.connect_neighbors(maze, row, col, "_dead-end_") # connect neighbors of dead end

                    # checking if it¡s either a *turn* or just an ordinary path
                    elif (paths == 2): # if there are two pathways around the point
                        crop = maze[row - 1:row + 2, col - 1:col + 2] # cropping the region around the point
                        nzero_loc = np.nonzero(crop > 0) # getting the location of the non-zero pixel
                        nzero_ptA = (nzero_loc[0][0], nzero_loc[1][0]) # getting the location of the first non-zero pixel
                        nzero_ptB = (nzero_loc[0][2], nzero_loc[1][2]) # getting the location of the second non-zero pixel
                        if not (((2 - nzero_ptA[0]) == nzero_ptB[0]) and ((2 - nzero_ptA[1]) == nzero_ptB[1])): # if the two non-zero pixels are not diagonally opposite
                            maze_bgr[row][col] = (255, 0, 0) # blue color

                            cv2.imshow("maze with the interest points", maze_bgr) # displaying the maze with interest points
                            # adding found vertex to graph as a turn
                            self.Graph.add_vertex((row, col), case = "_turn_") # adding vertex to graph
                            # connecting vertex to its neighbor (if there's any)
                            self.reset_connct_paramtrs() # reset state parameters of each vertex connection
                            self.connect_neighbors(maze, row, col, "_turn_") # connect neighbors of turn
                            turns += 1 # increment number of turns

                    # checking if it's either a *3-junc* or a *4-junc*
                    elif (paths > 2): # if there are more than 2 pathways around the point
                        if (paths == 3): # and if there are exactly 3 pathways around the point
                            maze_bgr[row][col] = (255, 244, 128) # orange color
                            if draw_interest_points: # if draw interest points flag is true
                                maze_bgr = self.triangle(maze_bgr, (col,row), 10, (0, 255, 0)) # draw a green triangle around the point
                            cv2.imshow("maze with the interest points", maze_bgr) # displaying the maze with interest points
                            # adding found vertex to graph as a 3-junction
                            self.Graph.add_vertex((row, col), case = "_3-junc_")
                            # connecting vertex to its neighbor (if there's any)
                            self.reset_connct_paramtrs() # reset state parameters of each vertex connection
                            self.connect_neighbors(maze, row, col, "_3-junc_") # connect neighbors of 3-junction
                            junc_3 += 1 # increment number of 3-junctions

                        else: # if there are more than 3 pathways around the point
                            maze_bgr[row][col] = (255, 0, 0) # blue
                            if draw_interest_points: # if draw interest points flag is true
                                cv2.rectangle(maze_bgr, (col - 10, row - 10) , (col + 10, row + 10), (255, 140, 144), 2)
                            cv2.imshow("maze with the interest points", maze_bgr)
                            # adding found vertex to graph as a 4-junction
                            self.Graph.add_vertex((row, col), case = "_4-junc_")
                            # connecting vertex to its neighbor (if there's any)
                            self.reset_connct_paramtrs() # reset state parameters of each vertex connection
                            self.connect_neighbors(maze, row, col, "_4-junc_") # connect neighbors of 4-junction
                            junc_4 += 1 # increment number of 4-junctions

        print("\ninterest points: \n[turns, 3_junc, 4_junc] [",turns,", ", junc_3,", ",junc_4,"]\n")

    # define a function to convert the maze to a graph for path planning
    def graphify(self, extracted_maze):
        # checking if graph extracted or not from the maze
        if not self.graphified: # if graph is not extracted from maze
            # input: ,aze occupancy grid from the localization stage
            cv2.imshow("maze extracted", extracted_maze)

            # first step: we'll be performing thinning on maze to reduce the area to paths that car could follow
            thinned = cv2.ximgproc.thinning(extracted_maze) # performing thinning on maze
            cv2.imshow('maze thinned', thinned) # displaying the thinned maze

            # second step: dilate and perform thinning again to minimize unneccesary interest point like turns
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2)) # creating a kernel for dilation
            thinned_dilated = cv2.morphologyEx(thinned, cv2.MORPH_DILATE, kernel) # dilating the thinned maze again
            _, bw2 = cv2.threshold(thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU) # thresholding the dilated maze
            thinned = cv2.ximgproc.thinning(bw2) # performing thinning on dilated maze
            cv2.imshow('maze thinned again', thinned) # displaying the thinned maze again

            # third step: crop out the boundary that is not part of maze to avoid false interest points
            thinned_cropped = thinned[self.crp_amt:thinned.shape[0] - self.crp_amt,
                                      self.crp_amt:thinned.shape[1] - self.crp_amt] # cropping the thinned maze
            cv2.imshow('cropped maze', thinned_cropped) # displaying the cropped maze

            # fourth step: overlay the found paths on maze occupancy grid
            extracted_maze_cropped = extracted_maze[self.crp_amt:extracted_maze.shape[0] - self.crp_amt,
                                                    self.crp_amt:extracted_maze.shape[1] - self.crp_amt] # cropping the maze
            extracted_maze_cropped = cv2.cvtColor(extracted_maze_cropped, cv2.COLOR_GRAY2BGR) # converting the cropped maze to colored
            extracted_maze_cropped[thinned_cropped > 0] = (0, 0, 255) # overlaying the thinned maze on cropped maze
            cv2.imshow('maze overlapped with the routes', extracted_maze_cropped) # displaying the overlapped maze

            # fifth step: identify interest points in the path to further reduce processing time
            self.one_pass(thinned_cropped) # find interest points in maze
            cv2.waitKey(0) # wait for user input

            self.maze = thinned_cropped # setting the maze to the thinned cropped maze
            self.graphified = True # setting graphified to true