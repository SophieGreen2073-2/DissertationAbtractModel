from AreaModel import AreaModel
from UAVModel import UAVModel
import numpy as np
from collections import deque

# Step the robot one step on the grid
def StepRobots(robots, area):
    for robot in robots:
        # Check if the robot should be moved
        if robot.steps_completed:
            continue

        # Get next step and start position
        cc, cr = robot.steps_queue.popleft()
        start = (robot.x_pos, robot.y_pos) 

        # If the next step is into a wall then clear the steps queue and move to next robot
        if robot.scanned_grid.grid[cr, cc] == 1:
            robot.steps_queue = deque()
            robot.steps_completed = True
            continue

        # Step robot into the next position
        step_dir = (cc - start[0], cr - start[1])
        robot.step(step_dir, area)

        # If this was the last step mark the robot as reached destination
        if len(robot.steps_queue) == 0:
            robot.steps_completed = True

    ShareRobotData(robots)


# Share the robot data between UAVs
# I hate this and want to change it but should work for now
def ShareRobotData(UAVs):
    for uav in UAVs:
        for uav2 in UAVs:
            if uav == uav2:
                continue
            for row in range(uav.scanned_grid.height):
                for col in range(uav.scanned_grid.width):
                    if uav.scanned_grid.grid[row, col] == 0 and (uav2.scanned_grid.grid[row, col] == uav2.robot_id or uav2.scanned_grid.grid[row, col] == 1):
                        uav.scanned_grid.grid[row, col] = uav2.scanned_grid.grid[row, col]


def main():
    area = AreaModel()
    area.BuildModel()
    area.DisplayStaticGrid()

    UAVs = [UAVModel(0,1,area,2, True), UAVModel(10, 1, area, 3, False)]
    # UAVs = [UAVModel(0,1,area,2)]
    # UAVs[0].scanned_grid.DisplayGrid()
    
    while any(uav.moved for uav in UAVs):
        for uav in UAVs:
            if uav.steps_completed:
                uav.yamauchi_move_create_full_frontier(area)
        # UAVs[0].scanned_grid.UpdateGrid()
        # ShareRobotData(UAVs)
        StepRobots(UAVs, area)
    

main()