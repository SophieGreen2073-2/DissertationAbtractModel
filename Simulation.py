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
        self.area.BuildModel(self.numUAVs)
        self.area.DisplayStaticGrid()

        self.UAVs = []

        for i in range(self.numUAVs):
            uav_start_position_x, uav_start_position_y = self.UAVStartPositions[i]
            DisplayGrid = i == 0
            self.UAVs.append(UAVModel(uav_start_position_x, uav_start_position_y, self.area, self.startRobotIDs + i, DisplayGrid, self.UAVParams["TopSpeed"], self.UAVParams["DangerSpeed"], self.UAVParams["StartSpeed"], self.UAVParams["LIDARDistance"], self.UAVParams["BatteryLife"]))

        while(True):
            for uav in self.UAVs:
                if uav.steps_completed:
                    uav.yamauchi_move_utility_function(self.area, self.startRobotIDs)
            self.StepRobots()


    # Step the robot one step on the grid
    def StepRobots(self):
        for robot in self.UAVs:
            robot.robot_next_step(self.startRobotIDs, self.time_step, self.area)

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


    # Get simulation parameters
    def GetParams(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, "SimulationParams.JSON")
        
        with open(json_path) as f:
            d = json.load(f)
            self.numUAVs = d["NumUAVs"]
            self.numUGVs = d["NumUGVs"]
            self.numLegged = d["NumLegged"]
            self.startRobotIDs = d["StartRobotIDs"]
            self.time_step = d["TimeStep"]
            self.UAVParams = d["UAVParams"]
            self.UAVStartPositions = self.UAVParams["StartPositions"]
            