import os
import json
from AreaModel import AreaModel
from RobotModels.UAVModel import UAVModel
import numpy as np
from collections import deque
from Record import RecordTime, RecordRedundancy

class Simulation():
    def __init__(self):
        print("Create Simulation")
        self.GetParams()

        self.area = AreaModel()

        for sim in self.simulations:
            num_uavs = sim["NumUAVs"]
            num_ugvs = sim["NumUGVs"]
            num_legged = sim["NumLegged"]

            self.area.BuildModel(num_uavs)
            # self.area.DisplayStaticGrid()

            self.UAVs = []

            self.time_elapsed = 0
            record_time = RecordTime()
            record_redundancy = RecordRedundancy()

            for i in range(num_uavs):
                DisplayGrid = i == 0
                # DisplayGrid = False
                self.UAVs.append(UAVModel(self.UAVParams["StartPosition"][0], self.UAVParams["StartPosition"][1], self.area, self.startRobotIDs + i, DisplayGrid, self.UAVParams["TopSpeed"], self.UAVParams["DangerSpeed"], self.UAVParams["StartSpeed"], self.UAVParams["LIDARDistance"], self.UAVParams["BatteryLife"], self.UAVParams["Acceleration"], self.UAVParams["WallDangerZone"], self.UAVParams["ChargeTime"]))

            while(True):
                self.completed = True
                for uav in self.UAVs:
                    if not uav.released and round(self.time_elapsed, 1) == round((uav.robot_id - self.startRobotIDs), 1) * self.UAVParams["ReleaseDelay"]:
                        uav.released = True
                    if uav.steps_completed and uav.released:
                        uav.yamauchi_move_utility_function(self.area, self.startRobotIDs)
                    self.completed &= uav.completed

                if self.completed:
                    break
                self.StepRobots()

            # record_time.record_time_elapsed(num_uavs, self.time_elapsed, self.UAVParams)
            # record_redundancy.record_overlap(self.area.overlap_area, num_uavs, self.UAVParams)


    # Step the robot one step on the grid
    def StepRobots(self):
        for robot in self.UAVs:
            robot.robot_next_step(self.startRobotIDs, self.time_step, self.area, self.time_step, self.recharge_point)

        # time.sleep(self.time_step)
        self.time_elapsed += self.time_step
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
            self.simulations = d["Simulations"]
            # self.numUAVs = d["NumUAVs"]
            # self.numUGVs = d["NumUGVs"]
            # self.numLegged = d["NumLegged"]
            self.startRobotIDs = d["StartRobotIDs"]
            self.time_step = d["TimeStep"]
            self.recharge_point = d["RechargePoint"]
            self.UAVParams = d["UAVParams"]
            # self.UAVStartPositions = self.UAVParams["StartPositions"]
            