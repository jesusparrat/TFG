'''
bot_pathplanning.py

- description:
node to perform path-planning from source to destination using provided methods.

- inputs:
1. graph extracted in mapping stage
2. source and destination
3. maze image
4. method to use DFS, DFS_Shortest, Dijsktra, A-star

- outputs:
1. self.path_to_goal        the computed path from source to destination (a list of coordinates)
2. self.img_shortest_path   the found path overlaid on image

- author:
Jesús Parra Torrijos | @jesusparrat

- date:
17/01/2024
'''

import cv2
import numpy as np
from numpy import sqrt

class bot_pathplanner(): # class to perform path-planning from source to destination using provided methods

    def __init__(self): # defining the class constructor (initialization method)

        self.DFS = DFS() # instance of DFS class
        self.dijsktra = dijsktra() # instance of dijsktra class
        self.astar = a_star() # instance of a_star class

        # state variables
        self.path_to_goal = [] # set the path to goal to empty list
        self.img_shortest_path = [] # set the image shortest path to empty list

    @staticmethod
    def cords_to_pts(cords): # convert coordinates to points
      return [cord[::-1] for cord in cords] # return the coordinates in reverse order

    def draw_path_on_maze(self, maze, shortest_path_pts, method): # draw the path on the maze
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR) # convert the maze to BGR
        rang = list(range(0,254,25)) # create a list of range from 0 to 254 with step 25

        depth = maze.shape[0] # get the maze depth

        for i in range(len(shortest_path_pts) - 1): # for each point in the shortest path
            per_depth = (shortest_path_pts[i][1]) / depth # compute the percentage of depth

            color = (
                      int(255 * (abs(per_depth + (-1 * (per_depth > 0.5))) *2)),
                      int(255 *  per_depth),
                      int(255 * (1 - per_depth))
                    ) # compute the color
            cv2.line(maze_bgr, shortest_path_pts[i], shortest_path_pts[i + 1], color) # draw the line

        img_str = "maze and path from " + method # set the image string
        cv2.namedWindow(img_str, cv2.WINDOW_FREERATIO) # create a named window
        cv2.imshow(img_str, maze_bgr) # show the image of the maze and the path
        self.img_shortest_path = maze_bgr   # save the image of the maze and the path

    def find_path_nd_display(self, graph, start, end, maze, method = "DFS"): # find the path from DFS and display it
        path_str = "path for " # set the path string

        if method == "DFS": # if the method is DFS
            paths = self.DFS.get_paths(graph, start, end) # get the paths
            path_to_display = paths[0] # get the path to display

        elif (method == "DFS_Shortest"): # if the method is DFS_Shortest
            paths_N_costs = self.DFS.get_paths_cost(graph, start, end) # get the paths and costs
            paths = paths_N_costs[0] # get the paths
            costs = paths_N_costs[1] # get the costs
            min_cost = min(costs) # get the minimum cost
            path_to_display = paths[costs.index(min_cost)] # get the path to display
            path_str = "shortest "+ path_str + "DFS"# set the path string

        elif (method == "dijsktra"): # if the method is Dijsktra
            if not self.dijsktra.shortestpath_found: # if the shortest path is not found
                print("finding the shortest routes") # print the message
                self.dijsktra.find_best_routes(graph, start, end) # find the best routes

            path_to_display = self.dijsktra.shortest_path # get the shortest path
            path_str = "shortest "+ path_str  + "Dijkstra" # set the path string

        elif (method == "a_star"): # if the method is A-star
            if not self.astar.shortestpath_found: # if the shortest path is not found
                print("finding the shortest routes") # print the message
                self.astar.find_best_routes(graph, start, end) # find the best routes

            path_to_display = self.astar.shortest_path # get the shortest path
            path_str = "\nshortest "+ path_str + "A*" # set the path string

        pathpts_to_display = self.cords_to_pts(path_to_display) # convert the path to display to points
        self.path_to_goal = pathpts_to_display # set the path to goal to the path to display

        print(path_str,"from {} to {} is =  {}".format(start, end, pathpts_to_display)) # print the path
        self.draw_path_on_maze(maze, pathpts_to_display, method) # draw the path on the maze
        cv2.waitKey(0) # wait for a key press

# DFS class to be used for DFS and DFS_Shortest
class DFS():
    # trying a recursive approach
    @staticmethod
    def get_paths(graph, start, end, path = []): # get the paths from start to end
        path = path + [start] # update the path to where ever you have been to

        if (start == end): # defining the simplest case
            return [path] # return the path

        if start not in graph.keys(): # handle boundary case that is the start not being part of graph
            return [] # return empty list

        paths = []  # list to store all possible paths from start to end

        # breakdown the complex problems into simpler sub-problems
        for node in graph[start].keys(): # for each node in the graph recursively call the problem with simpler case
            # once encountered base cond then roll back the answer to solver sub-problem
            if ((node not in path) and (node != "case")): # if the node is not already traversed and not a case key
                new_paths = DFS.get_paths(graph, node, end, path) # get the new paths
                for p in new_paths: # for each path in the new paths
                    paths.append(p) # insert the path into the paths
        return paths # return the paths

    # defining a function to get the paths and costs
    @staticmethod
    def get_paths_cost(graph,start,end,path=[],cost=0,trav_cost=0):

        # update the path and the cost to reaching that path
        path = path + [start]
        cost = cost + trav_cost

        # define the simplest case
        if start == end: # if the start is the end
            return [path], [cost] # return the path and the cost
        # handle boundary case that is the start not being part of graph
        if start not in graph.keys(): # if the start is not in the graph
            return [], 0 # return empty list of path and 0 cost

        paths = [] # list to store all possible paths from point A to B

        costs = [] # list to store costs of each possible path to goal

        for node in graph[start].keys(): # for each node in the graph retrieve all connections for that one damn node you are looking it

            # checking if not already traversed and not a "case" key
            if ((node not in path) and (node != "case")): # if the node is not already traversed and not a case key
                new_paths,new_costs = DFS.get_paths_cost(graph, node, end, path, cost, graph[start][node]['cost']) # get the new paths and costs

                for p in new_paths: # for each path in the new paths
                    paths.append(p) # insert the path into the paths
                for c in new_costs: # for each cost in the new costs
                    costs.append(c) # insert the cost into the costs
        return paths, costs # return the paths and the costs


# creating a class for minHeap to be used as a priority queue for dijsktra and A*
class Heap():

    def __init__(self): # defining the class constructor (initialization method)
        # priority queue will be stored in an array list of list containing vertex and their resp distance
        self.array = []

        self.size = 0 # counter to track nodes left in priority queue

        self.posOfVertices = [] # list to store the position of vertices

    # create a minheap node in a list of vertex and distance
    def new_minHeap_node(self, v, dist):
        return([v, dist]) # return the list of vertex and distance

    # swap node a with node b
    def swap_nodes(self, a, b):
        temp = self.array[a] # store node a in temp
        self.array[a] = self.array[b] # store node b in node a
        self.array[b] = temp # store temp in node b

    # convert any part of complete tree in minHeap in O(nlogn) time
    def minHeapify(self, node_idx): # node_idx is the index of the node to be heapified
        smallest = node_idx # initialize smallest as root
        left = (node_idx * 2) + 1 # left child
        right = (node_idx * 2) + 2 # right child

        if ((left < self.size) and (self.array[left][1] < self.array[smallest][1])): # if left child is smaller than root
            smallest = left # update the smallest
        if ((right < self.size) and (self.array[right][1] < self.array[smallest][1])): # if right child is smaller than root
            smallest = right # update the smallest

        if(smallest != node_idx): # if node_idx is not the smallest
            self.posOfVertices[self.array[node_idx][0]] = smallest # update the position of vertices
            self.posOfVertices[self.array[smallest][0]] = node_idx # update the position of vertices

            self.swap_nodes(node_idx, smallest) # swap the nodes

            self.minHeapify(smallest) # recursively call the minHeapify until all subnodes part of minheap or no more subnodes left

    # extract top (min value) node from the min-heap and then minheapify to keep heap property
    def extractmin(self):
        if self.size == 0: # handling boudary condtion
            return # return nothing

        root = self.array[0] # get the root node from the array extracted

        lastNode = self.array[self.size - 1] # move the last node on top
        self.array[0] = lastNode # move the last node on top

        self.posOfVertices[root[0]] = self.size - 1 # Update the postion of vertices
        self.posOfVertices[lastNode[0]] = 0 # Update the postion of vertices

        self.size -= 1# decrease the size of minheap by 1

        self.minHeapify(0) # Perform Minheapify from root

        return root # return extracted root node to user

    # update distance for a node to a new found shorter distance
    def decreaseKey(self, vertx, dist):
        idxofvertex = self.posOfVertices[vertx] # retreviing the idx of vertex we want to decrease value of

        self.array[idxofvertex][1] = dist # update the distance of the vertex

        # travel up while complete heap is not heapified
        while((idxofvertex > 0) and (self.array[idxofvertex][1] < self.array[(idxofvertex - 1) // 2][1])): # while idx is valid and updated key dist < parent key dist
            self.posOfVertices[self.array[idxofvertex][0]] = (idxofvertex - 1) // 2 # Update position of parent and curr_node
            self.posOfVertices[self.array[(idxofvertex-1) // 2][0]] = idxofvertex # Update position of parent and curr_node

            self.swap_nodes(idxofvertex, (idxofvertex-1)//2) # swap the nodes curr_node with parent

            idxofvertex = (idxofvertex-1)//2 # navigate to parent and start process again

    # utility function to check if a given vertex 'v' is in min heap or not
    def isInMinHeap(self, v):
        if self.posOfVertices[v] < self.size: # if the position of vertex is less than the size of minheap
            return True # return true
        return False # return false

# creating a class for dijsktra
class dijsktra():
    def __init__(self): # defining the class constructor (initialization method)
        # state variables
        self.shortestpath_found = False # set the shortest path found to false

        self.shortest_path = [] # once found save the shortest path


        self.minHeap = Heap() # instance variable assigned obj of heap class for implementing required priority queue


        self.idxs2vrtxs = {} # creating dictionaries to manage the world
        self.vrtxs2idxs = {} # creating dictionaries to manage the world

        self.dijiktra_nodes_visited = 0 # counter added to track total nodes visited to reach goal node


    def ret_shortestroute(self, parent, start, end, route): # function to return the shortest route
        route.append(self.idxs2vrtxs[end]) # keep updating the shortest route from end to start by visiting closest vertices starting fron end

        # once we have reached the start maze entry stop because we've found the shortest route
        if (end == start): # if the end is the start
            return

        end = parent[end] # update the end to the parent of the end

        self.ret_shortestroute(parent, start, end, route) # recursively call the function  with new end point until we reach start


    def find_best_routes(self, graph, start, end): # find the best routes from start to end
        # taking the first item of the list created by list comprehension which is while looping over the key value pair of graph and then return the pairs_idx that match the start key
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0] == start][0] # get the start index
        print("index of search key: {}".format(start_idx)) # print the start index

        dist = [] # distance list storing the distance of each node

        parent = [] # storing found shortest subpaths

        self.minHeap.size = len(graph.keys()) # set size of minHeap to be the total no of keys in the graph

        for idx,v in enumerate(graph.keys()): # for each index and vertex in the graph
            dist.append(1e7) # initialize dist for all vertices to inf
            # creating BinaryHeap by adding one node ([vrtx2idx(v),dist]) at a time to minHeap array so instead of vertex which is a tuple representing an interesting points we pass an index
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx])) # append the new minHeap node
            self.minHeap.posOfVertices.append(idx) # append the position of vertices

            parent.append(-1) # initialize parent_nodes_list with -1 for all indices

            self.vrtxs2idxs[v] = idx # updating dictionaries of vertices
            self.idxs2vrtxs[idx] = v # updating dictionaries of the positions of vertices

        dist[start_idx] = 0 # set the dist of reaching the start node to 0
        self.minHeap.decreaseKey(start_idx, dist[start_idx]) # decrease the key as new found dist of start_vertex is 0

        while(self.minHeap.size != 0): # loop as long as priority queue has nodes

            self.dijiktra_nodes_visited += 1 # counter added for keeping track of nodes visited to reach goal node

            curr_top = self.minHeap.extractmin() # retrieve the node with the min distance - the highest priority
            u_idx = curr_top[0] # get the index of the current top
            u = self.idxs2vrtxs[u_idx] # get the vertex of the current top

            for v in graph[u]: # check all neighbors of vertex u and update their distance if found shorter
                if v != "case": # ignore case node
                    print("vertex adjacent to {} is {}".format(u, v)) # print the vertex adjacent to the current vertex
                    v_idx = self.vrtxs2idxs[v] # get the index of the vertex

                    if (self.minHeap.isInMinHeap(v_idx) and (dist[u_idx] != 1e7) and ((graph[u][v]["cost"] + dist[u_idx]) < dist[v_idx])): # if we have not found shortest distance to v + new found cost2Node < known cost2node
                       dist[v_idx] = graph[u][v]["cost"] + dist[u_idx] # update the distance of the vertex
                       self.minHeap.decreaseKey(v_idx, dist[v_idx]) # decrease the key of the vertex
                       parent[v_idx] = u_idx # update the parent of the vertex

            # end condition: for when our end goal has already been visited, this means shortest part to end goal has already been found so break the loop
            if (u == end):
                break

        shortest_path = [] # shortest path list
        self.ret_shortestroute(parent, start_idx,self.vrtxs2idxs[end],shortest_path) # return the shortest route

        self.shortest_path = shortest_path[::-1] # return the shortest path from the beginning to the end
        self.shortestpath_found = True # set the shortest path found to true

# creating a class for a_star
class a_star(dijsktra):
    def __init__(self): # defining the class constructor (initialization method)

        super().__init__() # calling the constructor of the parent class

        self.astar_nodes_visited = 0 # set the astar nodes visited to 0

    # heuristic function: one of the components required to compute total cost of any node
    @staticmethod
    def euc_d(a,b): # euclidean distance
        return sqrt(pow((a[0] - b[0]), 2) + pow((a[1] - b[1]), 2)) # return the euclidean distance

    '''The Euclidean distance is the straight-line distance between two points in a space.
    This method assumes that a and b are tuples or lists of two elements each, representing the x and y coordinates of each point.

    The @staticmethod decorator indicates that this method is a static method,
    which means it belongs to the class it's defined in, not any instance of that class.
    You can call it on the class itself, not on an instance of the class.

    The method uses the pow function to square the difference in x-coordinates and the difference in y-coordinates.
    The pow function takes two arguments: a base and an exponent, and returns the base raised to the power of the exponent.
    In this case, it's used to square the differences in coordinates (since squaring is the same as raising to the power of 2).

    The sqrt function is then used to take the square root of the sum of these squared differences,
    which gives the Euclidean distance between the two points.'''


    def find_best_routes(self, graph, start, end): # find the best routes from start to end

        # taking the first item of the list created by list comprehension which is while looping over the key value pair of graph and then return the pairs_idx that match the start key
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0] == start][0] # get the start index
        print("index of search key : {}".format(start_idx)) # print the start index

        cost2node = [] # cost to node of reaching that node from start
        dist = [] # distance list storing the distance of each node
        parent = [] # storing found shortest subpaths

        self.minHeap.size = len(graph.keys()) # set the size of the minHeap to be the total number of keys in the graph

        for idx,v in enumerate(graph.keys()): # for each index and vertex in the graph
            cost2node.append(1e7) # initialize cost2node for all vertices to infinity

            dist.append(1e7) # initialize dist for all vertices to infinity
            # creating a binary-heap by adding one node ([vrtx2idx(v),dist]) at a time to minHeap array so instead of vertex which is a tuple representing an interest points we pass in an index
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx])) # append the new minHeap node
            self.minHeap.posOfVertices.append(idx) # append the position of vertices

            parent.append(-1) # initialize parent_nodes_list with -1 for all indices

            self.vrtxs2idxs[v] = idx # updating dictionaries of vertices
            self.idxs2vrtxs[idx] = v # updating dictionaries of the positions of vertices

        cost2node[start_idx] = 0 # setting cost of reaching start node to 0

        dist[start_idx] = cost2node[start_idx] + self.euc_d(start, end) # setting distance of reaching start node to 0 + heuristic cost of reaching end node from start node

        self.minHeap.decreaseKey(start_idx, dist[start_idx]) # decrease key as new found dist of start_vertex is 0

        while(self.minHeap.size!=0): # loop as long as priority queue has nodes
            self.astar_nodes_visited += 1 # counter added for keeping track of nodes visited to reach goal node

            curr_top = self.minHeap.extractmin() # retrieve the node with the min distance - the highest priority
            u_idx = curr_top[0] # get the index of the current top
            u = self.idxs2vrtxs[u_idx] # get the vertex of the current top

            for v in graph[u]: # for each vertex in the graph check all neighbors of vertex u and update their distance if found shorter
                if v != "case": # if the vertex is not a case
                    print("vertex adjacent to {} is {}".format(u,v)) # print the vertex adjacent to the current vertex
                    v_idx = self.vrtxs2idxs[v] # get the index of the vertex

                    if (self.minHeap.isInMinHeap(v_idx) and (dist[u_idx] != 1e7) and ((graph[u][v]["cost"] + cost2node[u_idx]) < cost2node[v_idx])): # if we have not found shortest distance to v + new found cost2Node < known cost2node
                       cost2node[v_idx] = graph[u][v]["cost"] + cost2node[u_idx] # update the cost to node of the vertex
                       dist[v_idx] = cost2node[v_idx] + self.euc_d(v, end) # update the distance of the vertex
                       self.minHeap.decreaseKey(v_idx, dist[v_idx]) # decrease the key of the vertex
                       parent[v_idx] = u_idx # update the parent of the vertex

            # end condition: for when our end goal has already been visited, this means shortest part to end goal has already been found so break the loop
            if (u == end):
                break

        shortest_path = [] # shortest path list
        self.ret_shortestroute(parent, start_idx, self.vrtxs2idxs[end], shortest_path) # return the shortest route

        self.shortest_path = shortest_path[::-1] # return the shortest path from the beginning to the end
        self.shortestpath_found = True # set the shortest path found to true


