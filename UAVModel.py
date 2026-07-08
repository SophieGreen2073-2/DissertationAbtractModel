import numpy as np
from AreaModel import AreaModel
import math
from collections import deque
from RobotModel import RobotModel

class UAVModel(RobotModel):
    def __init__(self, x, y, area: AreaModel, robot_id, DisplayGrid):
        print("Creating UAV")
        self.x_pos = x
        self.y_pos = y

        self.x_vel = 0
        self.y_vel = 0

        self.robot_id = robot_id

        # self.actions = ['north', 'south', 'east', 'west', 'stay']
        self.directions = {'north': [0, -1], 'south': [0, 1], 'east': [1, 0], 'west': [-1, 0], 'stay': [0,0]}

        self.scanned_grid = AreaModel()
        self.scanned_grid.height = area.height
        self.scanned_grid.width = area.width
        self.scanned_grid.grid = np.zeros((self.scanned_grid.height, self.scanned_grid.width))

        self.edge_positions = []
        self.moved = True
        self.DisplayGrid = DisplayGrid

        if self.DisplayGrid:
            self.scanned_grid.DisplayGrid()

        self.steps_queue = deque()
        self.steps_completed = True

        self.sensor_range = 5
        self.num_rays = 360
        self.FOV = 360

    # Simulate Lidar Scanning
    def simulate_lidar(self, area: AreaModel):
        start_angle = 0 - self.FOV/2
        angle_increment = self.FOV / (self.num_rays - 1)

        for i in range(self.num_rays):
            ray_angle = start_angle + (i * angle_increment)

            x_dir = np.cos(ray_angle)
            y_dir = np.sin(ray_angle)
            
            distance = 0
            step_size = 0.5

            while distance <= self.sensor_range:
                # Work out the current end point of the laser
                curr_x = self.x_pos + (x_dir * distance)
                curr_y = self.y_pos + (y_dir * distance)

                # Convert to a grid position
                grid_x = math.floor(curr_x)
                grid_y = math.floor(curr_y)

                distance = distance + step_size

                # If this laser has left the bound of the map move to the next one
                if grid_x < 0 or grid_x >= self.scanned_grid.width or grid_y < 0 or grid_y >= self.scanned_grid.height:
                    break

                # Check if the laser has hit an obstacle (wall)
                if area.grid[grid_y, grid_x] == 1:
                    self.scanned_grid.grid[grid_y, grid_x] = 1
                    break
                else:
                    self.scanned_grid.grid[grid_y, grid_x] = self.robot_id

            
        if self.DisplayGrid:
            self.scanned_grid.UpdateGrid()


    def yamauchi_move(self, area: AreaModel):
        # Want to find closest frontier position (unobserved space)
        # Ony want to look at all edges that are not visted directly next to visited
        # Implementing a breadth first search
        
        # Return current position if the position is not scanned
        dest_location = []

        directions = ['north', 'south', 'east', 'west']
        queue = deque([(self.x_pos, self.y_pos)])
        visited = [[False for _ in range(self.scanned_grid.width)] for _ in range(self.scanned_grid.height)]
        visited[self.y_pos][self.x_pos] = True
        parent = {(self.x_pos, self.y_pos): None}

        # Go through each position until there is an unknown space (frontier)
        
        # Could try and add something that checks walls etc. however maybe not because that is not the algorithm
        while len(queue) != 0:
            cc, cr = queue.popleft()

            if self.scanned_grid.grid[cr, cc] == 0:
                dest_location = (cc, cr)
                break

            for dir in directions:
                dr = self.directions[dir][1]
                dc = self.directions[dir][0]
                grid_val = self.scanned_grid.grid[cr + dr, cc + dc]

                if 0 <= cc + dc < self.scanned_grid.width and 0 <= cr + dr < self.scanned_grid.height and not visited[cr + dr][cc + dc] and grid_val != 1:
                    visited[cr + dr][cc + dc] = True
                    queue.append((cc + dc, cr + dr))
                    parent[(cc + dc, cr + dr)] = (cc, cr)
        
        # Change this to do all steps towards frontier instead of just one to reduce calculation
        if len(dest_location) != 0:
            # Find the next step the UAV should take to get to the free space selected
            next_step = dest_location
            while dest_location != (self.x_pos, self.y_pos):
                next_step = dest_location
                dest_location = parent[dest_location]

            step_dir = [next_step[0] - dest_location[0], next_step[1] - dest_location[1]]
            self.step(step_dir, area)
        else:
            self.moved = False


    # Check if the selected position is a frontier point (at least one discovered neigbour)
    def check_frontier(self, directions, cc, cr):
        if self.scanned_grid.grid[cr, cc] != 0:
            return False
        
        for dir in directions:
            dr = self.directions[dir][1]
            dc = self.directions[dir][0]
            check_c = cc + dc
            check_r = cr + dr
            if 0 <= check_c < self.scanned_grid.width and 0 <= check_r < self.scanned_grid.height:
                neighbour_val = self.scanned_grid.grid[check_r, check_c]
                if neighbour_val != 0 and neighbour_val != 1:
                    return True
        return False


    # Eucliden distance
    def heuristic_function(self, current_pos, target_pos):
        return ((target_pos[0] - current_pos[0])**2 + (target_pos[1] - current_pos[1])**2)**(1/2)


    # Create final path list to return once target has been reached
    def generate_path(self, preceeding_nodes, current):
        path = []
        while(current in preceeding_nodes):
            path.insert(0, current)
            current = preceeding_nodes[current]
        return path


    # A* algorithm
    def do_a_star(self, start, end, directions):
        # try:
            # Get the size of the grid
        open_nodes = [start]    # Currently open nodes, starting with the start node
        preceeding_nodes = {}   # Dictionary of the nodes preceeding the one selected, preceeding_nodes[n] = node that came before n in current cheapest path

        g_score = [[float('inf') for _ in range(self.scanned_grid.width)] for _ in range(self.scanned_grid.height)]  # Currently known cheapest path from start to n, set to infinity for every position initially
        f_score = [[float('inf') for _ in range(self.scanned_grid.width)] for _ in range(self.scanned_grid.height)]  # f_score(n) = g_score(n) + heuristic_function(n), representing current best guess of how cheap a path could be from start to finish through n, infinity for each position initially

        # Initialise the start position g_score and f_score
        g_score[start[1]][start[0]] = 0                 
        f_score[start[1]][start[0]] = self.heuristic_function(start, end)

        while(open_nodes):
            # Get open node with current lowest f score
            current_node = min(open_nodes, key=lambda node: f_score[node[1]][node[0]])

            # Check if the target has been reached, if so generate path and return
            if (current_node == end):
                return self.generate_path(preceeding_nodes, end)
            
            # Remove current node from open nodes
            open_nodes.remove(current_node)

            # Iterate through each direction from the current node
            for dir in directions:
                dx, dy = self.directions[dir]
                # Get position of neighbour in that direction
                neighbour_node = (current_node[0] + dx, current_node[1] + dy)

                # Check that the neighbour is within the grid and not a wall
                if (neighbour_node[0] < 0 or neighbour_node[1] < 0 or neighbour_node[0] >= self.scanned_grid.width or neighbour_node[1] >= self.scanned_grid.height or self.scanned_grid.grid[neighbour_node[1]][neighbour_node[0]] == 1):
                    continue

                # Calculate current g score for neighbour from selected node
                curr_g_score = g_score[current_node[1]][current_node[0]] + 1

                # If calculated g score is better than current g score for the selected neighbour, update position
                if (curr_g_score < g_score[neighbour_node[1]][neighbour_node[0]]):
                    # Set preceeding node to be current open node
                    preceeding_nodes[neighbour_node] = current_node

                    # Update g and f score for the selected neighbour
                    g_score[neighbour_node[1]][neighbour_node[0]] = curr_g_score
                    f_score[neighbour_node[1]][neighbour_node[0]] = curr_g_score + self.heuristic_function(neighbour_node, end)

                    # Add neighbour to open nodes if not already there
                    if(neighbour_node not in open_nodes):
                        open_nodes.append(neighbour_node)
        # except Exception as e:
            # print(e)

        return []


    # Frontier based search
    def yamauchi_move_create_full_frontier(self, area: AreaModel):
        dest_location = []

        directions = ['north', 'south', 'east', 'west']
        queue = deque([(self.x_pos, self.y_pos)])

        MapOpenList = {(self.x_pos, self.y_pos)}
        MapCloseList = set()
        FrontierOpenList = set()
        FrontierCloseList = set()

        # Check if the current position of the UAV is unscanned
        if self.scanned_grid.grid[self.y_pos, self.x_pos] == 0:
            dest_location = (self.x_pos, self.y_pos)
            area.grid[self.y_pos, self.x_pos] = self.robot_id
            # self.one_step_scan(area)
            self.simulate_lidar(area)
            return

        # Go through each position until there is an unknown space (frontier)
        while len(queue) != 0 and len(dest_location) == 0:
            cc, cr = queue.popleft()

            # If p has not been visited
            if (cc, cr) in MapCloseList or self.scanned_grid.grid[cr, cc] == 1:
                continue

            is_frontier = False
            # If p is a frontier point
            if self.scanned_grid.grid[cr, cc] == 0:
                is_frontier = self.check_frontier(directions, cc, cr)

            if is_frontier:
                # Add p to the frontier queue
                queue_frontier = deque([(cc, cr)])
                NewFrontier = []
                FrontierOpenList.add((cc, cr))

                # While there are frontier points that have not been checked
                while len(queue_frontier) != 0:
                    # Pick unchecked frontier point
                    fc, fr = queue_frontier.popleft()
                    # If q has not been checked
                    if (fc, fr) in MapCloseList or (fc, fr) in FrontierCloseList:
                        continue
                    
                    # Check if the point in the queue is a frontier point
                    frontier_point = self.check_frontier(directions, fc, fr)

                    # If point in frontier fcheck list is a frontier point
                    if frontier_point:
                        NewFrontier.append((fc, fr))
                        
                        # Check all adjacent points to the frontier
                        for dir in directions:
                            dr = self.directions[dir][1]
                            dc = self.directions[dir][0]
                            w = (fc + dc, fr + dr)
                            
                            # If w is not checked then add it to the queue
                            if 0 <= w[0] < self.scanned_grid.width and 0 <= w[1] < self.scanned_grid.height:
                                if w not in FrontierOpenList and w not in FrontierCloseList and w not in MapCloseList:
                                    queue_frontier.append(w)
                                    FrontierOpenList.add(w)
                    FrontierCloseList.add((fc, fr))

                # Close the current point
                for p in NewFrontier:
                    MapCloseList.add(p)

                # Find centroid in New Frontier list
                total_x, total_y = 0, 0
                for val in NewFrontier:
                    total_x += val[0]
                    total_y += val[1]

                x_target = total_x // len(NewFrontier)
                y_target = total_y // len(NewFrontier)
                dest_location = (x_target, y_target)
                break

            # Add adjacent points to the check queue
            for dir in directions:
                dr = self.directions[dir][1]
                dc = self.directions[dir][0]
                adj_point = (cc + dc, cr + dr)

                # Check if each adjacent point has not been checked and is within bounds
                if 0<= adj_point[0] < self.scanned_grid.width and 0 <= adj_point[1] < self.scanned_grid.height:
                    if adj_point not in MapOpenList and adj_point not in MapCloseList:
                        
                        if self.scanned_grid.grid[adj_point[1], adj_point[0]] != 1:
                            queue.append(adj_point)
                            MapOpenList.add(adj_point)
            
            MapCloseList.add((cc, cr))

        # Generate path to target
        path = self.do_a_star((self.x_pos, self.y_pos), dest_location, directions)

        for step in path:
            self.steps_queue.append(step)

        if len(path) != 0:
            self.steps_completed = False


    # Move UAV into new position
    def step(self, step_val, area: AreaModel):
        if area.grid[self.y_pos + step_val[1], self.x_pos + step_val[0]] != 1:
            self.x_pos += step_val[0]
            self.y_pos += step_val[1]
            area.grid[self.y_pos, self.x_pos] = self.robot_id
            # self.one_step_scan(area)
            self.simulate_lidar(area)


    # Scan the area around the UAV (1 space in each direction)
    def one_step_scan(self, area: AreaModel):
        for dir in self.directions:
            dir_val = self.directions[dir]
            x = self.x_pos + dir_val[0]
            y = self.y_pos + dir_val[1]
            area.area_scan(self.robot_id, x, y)
            self.scanned_grid.grid[y, x] = area.grid[y, x]
        if self.DisplayGrid:
            self.scanned_grid.UpdateGrid()


    