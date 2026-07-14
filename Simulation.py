import os
import json
from AreaModel import AreaModel
from RobotModels.UAVModel import UAVModel
import numpy as np
from collections import deque

class Simulation():
    def __init__(self):
        print("Create Simulation")
        self.GetParams()

        self.area = AreaModel()
        self.area.BuildModel(self.numUAVsSIm)
        self.area.DisplayStaticGrid()

        self.UAVs = []

        for i in range(self.numUAVs):
            uav_start_position_x, uav_start_position_y = self.UAVStartPositions[i]
            DisplayGrid = i == 0
            self.UAVs.append(UAVModel(uav_start_position_x, uav_start_position_y, self.area, self.startRobotIDs + i, DisplayGrid))

        while(True):
            for uav in self.UAVs:
                if uav.steps_completed:
                    uav.yamauchi_move_utility_function(self.area)
            self.StepRobots()


    # Step the robot one step on the grid
    def StepRobots(self):
        for robot in self.UAVs:
            # Check if the robot should be moved
            if robot.steps_completed:
                continue

            # Get next step and start position
            cc, cr = robot.steps_queue.popleft()
            start = (robot.x_pos, robot.y_pos) 

            # If the next step is into a wall then clear the steps queue and move to next robot
            if robot.scanned_grid.grid[cr, cc] == 1:
                robot.steps_queue.clear()
                robot.steps_completed = True
                continue

            # Step robot into the next position
            step_dir = (cc - start[0], cr - start[1])
            robot.step(step_dir, self.area)

            # If this was the last step mark the robot as reached destination
            if len(robot.steps_queue) == 0:
                robot.steps_completed = True

        self.ShareRobotData()


    # Share the robot data between UAVs
    # I hate this and want to change it but should work for now
    # Need to consider if the robots can actually communicatr with each other where they currently are
    # Probably need a new model or maybe in the area model
    def ShareRobotData(self):
        for uav in self.UAVs:
            uav.localUAVs = []
            for uav2 in self.UAVs:
                if uav == uav2:
                    continue
                else:
                    if not uav2 in uav.localUAVs:
                        uav.localUAVs.append(uav2)
                for row in range(uav.scanned_grid.height):
                    for col in range(uav.scanned_grid.width):
                        if uav.scanned_grid.grid[row, col] == 0 and (uav2.scanned_grid.grid[row, col] == uav2.robot_id or uav2.scanned_grid.grid[row, col] == 1):
                            uav.scanned_grid.grid[row, col] = uav2.scanned_grid.grid[row, col]


    def GetParams(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, "SimulationParams.JSON")
        
        with open(json_path) as f:
            d = json.load(f)
            self.numUAVs = d["NumUAVs"]
            self.numUGVs = d["NumUGVs"]
            self.numLegged = d["NumLegged"]
            self.startRobotIDs = d["StartRobotIDs"]
            self.UAVStartPositions = d["UAVStartPositions"]