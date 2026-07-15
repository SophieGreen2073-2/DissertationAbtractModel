from AreaModel import AreaModel
from collections import deque
import numpy as np
import math

class RobotModel:
    def __init__(self, x, y, robot_id, area: AreaModel, DisplayGrid, top_speed, danger_speed, start_speed, lidar_scan_distance, battery_life):
        print("New Robot")
        # Robot position
        self.x_pos = float(x)
        self.y_pos = float(y)

        # Robot Velocity
        self.top_speed = top_speed
        self.danger_speed = danger_speed
        self.start_speed = start_speed

        # Robot battery life
        self.battery_life = battery_life

        # Robot ID
        self.robot_id = robot_id

        # Robot Directions
        self.directions = {'north': [0, -1], 'south': [0, 1], 'east': [1, 0], 'west': [-1, 0], 'stay': [0,0], 'north_east': [1, -1], 'south_east': [1, 1], 'south_west': [-1, 1], 'north_west': [-1, -1]}

        # Robot area grid belief
        self.scanned_grid = AreaModel()
        self.scanned_grid.height = area.height
        self.scanned_grid.width = area.width
        self.scanned_grid.grid = np.zeros((self.scanned_grid.height, self.scanned_grid.width))
        
        # Abstract model display
        self.DisplayGrid = DisplayGrid
        if self.DisplayGrid:
            self.scanned_grid.DisplayGrid()

        # Robot Movement Steps queue
        self.steps_queue = deque()
        self.steps_completed = True
        self.target = None

        # Robot sensor information
        self.sensor_range = lidar_scan_distance
        self.num_rays = 360
        self.FOV = 360


    # Get grid position
    def get_grid_pos(self):
        return (int(math.floor(self.x_pos)), int(math.floor(self.y_pos)))


    # Eucliden distance
    def heuristic_function(self, current_pos, target_pos):
        return ((target_pos[0] - current_pos[0])**2 + (target_pos[1] - current_pos[1])**2)**(1/2)


    # Create final path list to return once target has been reached
    def generate_path(self, preceeding_nodes, current):
        while(current in preceeding_nodes):
            # path.insert(0, current)
            self.steps_queue.appendleft(current)
            current = preceeding_nodes[current]
    

    # A* algorithm
    def do_a_star(self, start, end):
        # Get the size of the grid
        open_nodes = [start]    # Currently open nodes, starting with the start node
        preceeding_nodes = {}   # Dictionary of the nodes preceeding the one selected, preceeding_nodes[n] = node that came before n in current cheapest path

        g_score = [[float('inf') for _ in range(self.scanned_grid.width)] for _ in range(self.scanned_grid.height)]  # Currently known cheapest path from start to n, set to infinity for every position initially
        f_score = [[float('inf') for _ in range(self.scanned_grid.width)] for _ in range(self.scanned_grid.height)]  # f_score(n) = g_score(n) + heuristic_function(n), representing current best guess of how cheap a path could be from start to finish through n, infinity for each position initially

        # Initialise the start position g_score and f_score
        g_score[start[1]][start[0]] = 0                 
        f_score[start[1]][start[0]] = self.heuristic_function(start, end)

        directions = ['north', 'south', 'east', 'west']

        while(open_nodes):
            # Get open node with current lowest f score
            current_node = min(open_nodes, key=lambda node: f_score[node[1]][node[0]])

            # Check if the target has been reached, if so generate path and return
            if (current_node == end):
                self.generate_path(preceeding_nodes, end)
                return
            
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


    # Work out the next robot step 
    def robot_next_step(self, start_robot_ids, dt, area):
        # Check if the robot should be moved
        if self.steps_completed:
            return

        # Get next step and start position
        # cc, cr = robot.steps_queue.popleft()
        start = (self.x_pos, self.y_pos) 
        if not self.target:
            self.target = self.steps_queue.popleft()

        # Step robot into the next position
        step_dir = (self.target[0] - start[0], self.target[1] - start[1])
        distance = np.hypot(step_dir[0], step_dir[1])

        if distance < 0.1:
            self.target = self.steps_queue.popleft()

        # If the next step is into a wall then clear the steps queue and move to next robot
        if self.scanned_grid.grid[self.target[1], self.target[0]] == 1:
            self.steps_queue.clear()
            self.steps_completed = True
            return

        # Check if the robot is entering a tight space so should be at a slower speed

        # Accelerate/decelerate

        step_distance = self.top_speed * dt
        if step_distance >= distance:
            self.x_pos, self.y_pos = float(self.target[0]), float(self.target[1])
            self.steps_completed = True
        else:
            self.x_pos += (step_dir[0] / distance) * step_distance
            self.y_pos += (step_dir[1] / distance) * step_distance
        # self.step(step_dir, self.area, start_robot_ids)
        self.simulate_lidar(area, start_robot_ids)

        # If this was the last step mark the robot as reached destination
        if len(self.steps_queue) == 0:
            self.steps_completed = True

    
    # Move robot into new position
    def step(self, step_val, area: AreaModel, robot_start_id):
        current_grid_pos = self.get_grid_pos()

        if area.grid[current_grid_pos[1] + step_val[1], current_grid_pos[0] + step_val[0]] != 1:
            self.x_pos += step_val[0]
            self.y_pos += step_val[1]
            self.simulate_lidar(area, robot_start_id)


    # Simulate Lidar Scanning for UAV
    def simulate_lidar(self, area: AreaModel, robot_start_id):
        start_angle = 0 - self.FOV/2
        angle_increment = self.FOV / (self.num_rays - 1)

        for i in range(self.num_rays):
            ray_angle = start_angle + (i * angle_increment)

            x_dir = np.cos(ray_angle)
            y_dir = np.sin(ray_angle)
            
            distance = 0
            step_size = 0.5
            
            grid_x = -1
            grid_y = -1

            while distance <= self.sensor_range:
                # Work out the current end point of the laser
                curr_x = self.x_pos + (x_dir * distance)
                curr_y = self.y_pos + (y_dir * distance)

                # Convert to a grid position
                temp_grid_x = math.floor(curr_x)
                temp_grid_y = math.floor(curr_y)

                # If this laser has left the bound of the map move to the next one
                if temp_grid_x < 0 or temp_grid_x >= self.scanned_grid.width or temp_grid_y < 0 or temp_grid_y >= self.scanned_grid.height:
                    break

                # Mark the grid position as scanned by the robot scanning it
                if temp_grid_x != grid_x or temp_grid_y != grid_y:
                    grid_x = temp_grid_x
                    grid_y = temp_grid_y
                    area.overlap_area[grid_y, grid_x, self.robot_id - robot_start_id]

                distance += step_size

                # Check if the laser has hit an obstacle (wall)
                if area.grid[grid_y, grid_x] == 1:
                    self.scanned_grid.grid[grid_y, grid_x] = 1
                    break
                else:
                    self.scanned_grid.grid[grid_y, grid_x] = self.robot_id

            
        if self.DisplayGrid:
            self.scanned_grid.UpdateGrid()


