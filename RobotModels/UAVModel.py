import numpy as np
from AreaModel import AreaModel
import math
from collections import deque
from RobotModels.RobotModel import RobotModel

class UAVModel(RobotModel):
    def __init__(self, x, y, area: AreaModel, robot_id, DisplayGrid, top_speed, danger_speed, start_speed, lidar_distance, battery_life, acceleration, wall_danger_zone, charge_time):
        print("Creating UAV")
        RobotModel.__init__(self, x, y, robot_id, area, DisplayGrid, top_speed, danger_speed, start_speed, lidar_distance, battery_life, acceleration, wall_danger_zone, charge_time)

        self.type = "UAV"
        self.frontier_count = 20
        self.localUAVs = []
        self.util_cost_weight = 1
        self.util_penalty_weight = 300
        self.util_wall_weight = 10
        self.released = False


    # Basic yamauchi move (move to the closest free square, no search for frontiers)
    def yamauchi_move(self, area: AreaModel, robot_start_id):
        # Want to find closest frontier position (unobserved space)
        # Ony want to look at all edges that are not visted directly next to visited
        # Implementing a breadth first search
        
        # Return current position if the position is not scanned
        dest_location = []

        current_grid_pos = self.get_grid_pos()

        directions = ['north', 'south', 'east', 'west']
        queue = deque([current_grid_pos])
        visited = [[False for _ in range(self.scanned_grid.width)] for _ in range(self.scanned_grid.height)]
        visited[current_grid_pos[1]][current_grid_pos[0]] = True
        parent = {current_grid_pos: None}

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
            while dest_location != current_grid_pos:
                next_step = dest_location
                dest_location = parent[dest_location]

            step_dir = [next_step[0] - dest_location[0], next_step[1] - dest_location[1]]
            self.step(step_dir, area, robot_start_id)
        else:
            self.moved = False


    def build_frontier(self, queue_frontier, MapCloseList, FrontierCloseList, directions, NewFrontier, FrontierOpenList, current_grid_pos):
        # While there are frontier points that have not been checked
        while len(queue_frontier) != 0:
            # Pick unchecked frontier point
            fc, fr = queue_frontier.popleft()
            # If q has not been checked
            if (fc, fr) in MapCloseList or (fc, fr) in FrontierCloseList:
                continue
            
            # Check if the point in the queue is a frontier point
            frontier_point = self.check_frontier(directions, fc, fr)
            distance = self.heuristic_function(current_grid_pos, (fc, fr))
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
            close_to_wall = self.check_close_wall()
            # for dir in directions:
            #     dir_val = self.directions[dir]
            #     for i in range(self.wall_danger_zone):
            #         # Get position we are checking for wall
            #         scaled_dir_val = tuple(item * (i+1) for item in dir_val)
            #         curr_x = p[0] + scaled_dir_val[0]
            #         curr_y = p[1] + scaled_dir_val[1]

            #         # Chceck if selected position is within grid bounds
            #         if curr_x < 0 or curr_x >= self.scanned_grid.width or curr_y < 0 or curr_y >= self.scanned_grid.height:
            #             break

            #         # Check grid position
            #         grid_val = self.scanned_grid.grid[curr_y, curr_x]
            #         if grid_val == 1:
            #             close_to_wall = True
            #             break

            # Move the UAV away from the wall if within 3 squares
            if not close_to_wall:
                new_frontier_away_from_walls.append(p)
            MapCloseList.add(p)

        # If there are no points in the frontier away from the wall use the normal frontier
        if len(new_frontier_away_from_walls) == 0:
            new_frontier_away_from_walls = NewFrontier

        return MapCloseList, FrontierCloseList, FrontierOpenList, NewFrontier, new_frontier_away_from_walls
    

    def utility_function(self, p, directions, current_grid_pos):
        # Current drones distance to the frontier point
        current_to_p = self.heuristic_function(current_grid_pos, p)
        if current_to_p == 0:
            current_to_p = 0.1

        cost = current_to_p * self.util_cost_weight

        # Distance of frontier point
        walls_to_p = float('inf')

        # Check each direction for a wall to penalise wall distance
        for dir in directions:
            dir_val = self.directions[dir]
            for i in range(self.wall_danger_zone):
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
                    if walls_to_p == float('inf'):
                        walls_to_p = 0
                    walls_to_p += self.heuristic_function((curr_x, curr_y), p)
                    break
        
        if walls_to_p == float('inf') or walls_to_p == 0:
            wall_penalty = 0
        else:
            # Closer to wall = larger penalty
            wall_penalty = self.util_wall_weight / (walls_to_p ** 2)

        uav_penalty = 0
        # Calculate distance to frontier point from each other UAV
        for uav in self.localUAVs:
            uav_to_p = self.heuristic_function((uav.x_pos, uav.y_pos), p)
            if uav_to_p == 0:
                uav_to_p = 0.1
                
            # If another UAV is closer to this target than we are, heavily penalize it
            if uav_to_p < current_to_p:
                # Scale penalty exponentially when close to minimize shared frontiers
                uav_penalty += self.util_penalty_weight / (uav_to_p ** 2)
            else:
                uav_penalty += (self.util_penalty_weight * 0.5) / (uav_to_p ** 2)

        return -cost - wall_penalty - uav_penalty


    # Yamauchi frontier algorithm that uses a utility function to choose the target point
    def yamauchi_move_utility_function(self, area: AreaModel, robot_start_id):
        current_grid_pos = self.get_grid_pos()
        dest_location = tuple()
        frontiers_found = []
        directions = ['north', 'south', 'east', 'west', 'north_east', 'south_east', 'south_west', 'north_west']
        queue = deque([current_grid_pos])

        MapOpenList = {current_grid_pos}
        MapCloseList = set()

        # Check if the current position of the UAV is unscanned
        if self.scanned_grid.grid[current_grid_pos[1], current_grid_pos[0]] == 0:
            area.grid[current_grid_pos[1], current_grid_pos[0]] = self.robot_id
            self.simulate_lidar(area, robot_start_id)
            return
        
        # Go through each position until frontier found
        while len(queue) != 0 and len(frontiers_found) <= self.frontier_count:
            cc, cr = queue.popleft()

            # If p has not been visited
            if (cc, cr) in MapCloseList or self.scanned_grid.grid[cr, cc] == 1:
                continue
            
            if (cc, cr) != current_grid_pos:
                # If p is a frontier point
                is_frontier = self.check_frontier(directions, cc, cr)

                # If the point is a frontier point add to the list of frontiers
                if is_frontier:
                    frontiers_found.append((cc, cr))

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

        best_cost_val = float('-inf')

        # Check if the chose frontier point is close to walls
        # new_frontier_away_from_walls = []
        # for p in frontiers_found:
        #     for dir in directions:
        #         dir_val = self.directions[dir]
        #         for i in range(self.sensor_range):
        #             # Get position we are checking for wall
        #             scaled_dir_val = tuple(item * (i+1) for item in dir_val)
        #             curr_x = p[0] + scaled_dir_val[0]
        #             curr_y = p[1] + scaled_dir_val[1]

        #             # Chceck if selected position is within grid bounds
        #             if curr_x < 0 or curr_x >= self.scanned_grid.width or curr_y < 0 or curr_y >= self.scanned_grid.height:
        #                 break

        #             # Check grid position
        #             grid_val = self.scanned_grid.grid[curr_y, curr_x]
        #             if grid_val == 1:
        #                 close_to_wall = True
        #                 break

        #     # Move the UAV away from the wall if within 3 squares
        #     if not close_to_wall:
        #         new_frontier_away_from_walls.append(p)

        # if len(new_frontier_away_from_walls) == 0:
        #     new_frontier_away_from_walls = frontiers_found

        for p in frontiers_found:
            util_val = self.utility_function(p, directions, current_grid_pos)

            if util_val > best_cost_val:
                best_cost_val = util_val
                dest_location = p

        if len(dest_location) == 0:
            self.completed = True

        # Generate path to target
        self.do_a_star(current_grid_pos, dest_location, True)
        # self.target = dest_location

        if len(self.steps_queue) != 0:
            # self.scanned_grid.grid[dest_location[1], dest_location[0]] = 2
            self.steps_completed = False


    # Frontier based search
    def yamauchi_move_create_full_frontier(self, area: AreaModel, robot_start_id):
        current_grid_pos = self.get_grid_pos()
        
        dest_location = []

        directions = ['north', 'south', 'east', 'west', 'north_east', 'south_east', 'south_west', 'north_west']
        queue = deque([current_grid_pos])

        MapOpenList = {current_grid_pos}
        MapCloseList = set()
        FrontierOpenList = set()
        FrontierCloseList = set()

        # Check if the current position of the UAV is unscanned
        if self.scanned_grid.grid[current_grid_pos[1], current_grid_pos[0]] == 0:
            dest_location = current_grid_pos
            area.grid[current_grid_pos[1], current_grid_pos[0]] = self.robot_id
            # self.one_step_scan(area)
            self.simulate_lidar(area, robot_start_id)
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
                    queue_frontier, MapCloseList, FrontierCloseList, directions, NewFrontier, FrontierOpenList, current_grid_pos)

                # Find centroid in New Frontier list
                total_x, total_y = 0, 0
                for val in new_frontier_away_from_walls:
                    total_x += val[0]
                    total_y += val[1]

                x_target = total_x // len(new_frontier_away_from_walls)
                y_target = total_y // len(new_frontier_away_from_walls)
                dest_location = (x_target, y_target)

                # If centroid is the current position then send to first discovered frontier point (should be closest one)
                if dest_location == current_grid_pos or self.scanned_grid.grid[y_target, x_target] == 1:
                    dest_location = next(
                        (p for p in new_frontier_away_from_walls if p != current_grid_pos), 
                        new_frontier_away_from_walls[0]  # Fallback value if every single point matches current_loc
                    )

                    if dest_location == current_grid_pos:
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
                                key=lambda p: (p[0] - current_grid_pos[0])**2 + (p[1] - current_grid_pos[0])**2
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
        self.do_a_star(current_grid_pos, dest_location, True)

        if len(self.steps_queue) != 0:
            self.scanned_grid.grid[dest_location[1], dest_location[0]] = 2
            self.steps_completed = False



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


    