import numpy as np
from AreaModel import AreaModel
import math
from collections import deque
from RobotModels.RobotModel import RobotModel

class UAVModel(RobotModel):
    def __init__(self, x, y, area: AreaModel, robot_id, DisplayGrid):
        print("Creating UAV")
        RobotModel.__init__(self, x, y, robot_id, area, DisplayGrid)

        self.type = "UAV"


    # Simulate Lidar Scanning for UAV
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

                distance += step_size

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


    # Basic yamauchi move (move to the closest free square, no search for frontiers)
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
        if self.scanned_grid.grid[cr, cc] == 0 or self.scanned_grid.grid[cr, cc] == 1:
            return False
        
        for dir in directions:
            dr = self.directions[dir][1]
            dc = self.directions[dir][0]
            check_c = cc + dc
            check_r = cr + dr
            if 0 <= check_c < self.scanned_grid.width and 0 <= check_r < self.scanned_grid.height:
                neighbour_val = self.scanned_grid.grid[check_r, check_c]
                if neighbour_val == 0:
                    if not self.check_corner((cc, cr), (check_c, check_r)):
                        return True
        
        return False
    

    def build_frontier(self, queue_frontier, MapCloseList, FrontierCloseList, directions, NewFrontier, FrontierOpenList):
        # While there are frontier points that have not been checked
        while len(queue_frontier) != 0:
            # Pick unchecked frontier point
            fc, fr = queue_frontier.popleft()
            # If q has not been checked
            if (fc, fr) in MapCloseList or (fc, fr) in FrontierCloseList:
                continue
            
            # Check if the point in the queue is a frontier point
            frontier_point = self.check_frontier(directions, fc, fr)
            distance = self.heuristic_function((self.x_pos, self.y_pos), (fc, fr))
            if distance > self.sensor_range and len(NewFrontier) != 0:
                frontier_point = False

            # If point in frontier check list is a frontier point
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
        new_frontier_away_from_walls = []
        for p in NewFrontier:
            # Check if the point in the frontier is too close to a wall
            close_to_wall = False
            for dir in directions:
                dir_val = self.directions[dir]
                for i in range(self.sensor_range):
                    # Get position we are checking for wall
                    scaled_dir_val = tuple(item * (i+1) for item in dir_val)
                    curr_x = p[0] + scaled_dir_val[0]
                    curr_y = p[1] + scaled_dir_val[1]

                    # Chceck if selected position is within grid bounds
                    if curr_x < 0 or curr_x >= self.scanned_grid.width or curr_y < 0 or curr_y >= self.scanned_grid.height:
                        break

                    # Check grid position
                    grid_val = self.scanned_grid.grid[curr_y, curr_x]
                    if grid_val == 1:
                        close_to_wall = True
                        break

            # Move the UAV away from the wall if within 3 squares
            if not close_to_wall:
                new_frontier_away_from_walls.append(p)
            MapCloseList.add(p)

        # If there are no points in the frontier away from the wall use the normal frontier
        if len(new_frontier_away_from_walls) == 0:
            new_frontier_away_from_walls = NewFrontier

        return MapCloseList, FrontierCloseList, FrontierOpenList, NewFrontier, new_frontier_away_from_walls


    # Check if the frontiers lateral free space is behind a corner
    def check_corner(self, pos, free_pos):
        wall_x, wall_y = pos[0], pos[1]
        free_x, free_y = free_pos[0], free_pos[1]

        corner_a = self.scanned_grid.grid[free_y, wall_x]
        corner_b = self.scanned_grid.grid[wall_y, free_x]
        
        if corner_a == 1 and corner_b == 1:
            return True
            
        return False
                

    # Frontier based search
    def yamauchi_move_create_full_frontier(self, area: AreaModel):
        dest_location = []

        directions = ['north', 'south', 'east', 'west', 'north_east', 'south_east', 'south_west', 'north_west']
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

            # If p is a frontier point
            is_frontier = self.check_frontier(directions, cc, cr)

            if is_frontier:
                # Add p to the frontier queue
                queue_frontier = deque([(cc, cr)])
                NewFrontier = []
                FrontierOpenList.add((cc, cr))

                MapCloseList, FrontierCloseList, FrontierOpenList, NewFrontier, new_frontier_away_from_walls = self.build_frontier(
                    queue_frontier, MapCloseList, FrontierCloseList, directions, NewFrontier, FrontierOpenList)

                # Find centroid in New Frontier list
                total_x, total_y = 0, 0
                for val in new_frontier_away_from_walls:
                    total_x += val[0]
                    total_y += val[1]

                x_target = total_x // len(new_frontier_away_from_walls)
                y_target = total_y // len(new_frontier_away_from_walls)
                dest_location = (x_target, y_target)

                # If centroid is the current position then send to first discovered frontier point (should be closest one)
                if dest_location == (self.x_pos, self.y_pos) or self.scanned_grid.grid[y_target, x_target] == 1:
                    dest_location = next(
                        (p for p in new_frontier_away_from_walls if p != (self.x_pos, self.y_pos)), 
                        new_frontier_away_from_walls[0]  # Fallback value if every single point matches current_loc
                    )

                    if dest_location == (self.x_pos, self.y_pos):
                        # Run a raw, unfiltered search to find the absolute closest open frontier cell
                        raw_frontier_backup = []
                        for r in range(self.scanned_grid.height):
                            for c in range(self.scanned_grid.width):
                                if self.check_frontier(directions, c, r):
                                    raw_frontier_backup.append((c, r))
                        
                        if raw_frontier_backup:
                            # Target the closest raw frontier cell, ignoring the wall safety padding
                            dest_location = min(
                                raw_frontier_backup,
                                key=lambda p: (p[0] - self.x_pos)**2 + (p[1] - self.y_pos)**2
                            )
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
        self.do_a_star((self.x_pos, self.y_pos), dest_location)
        self.target = dest_location
        if len(self.steps_queue) != 0:
            self.steps_completed = False


    # Move UAV into new position
    def step(self, step_val, area: AreaModel):
        if area.grid[self.y_pos + step_val[1], self.x_pos + step_val[0]] != 1:
            self.x_pos += step_val[0]
            self.y_pos += step_val[1]
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


    