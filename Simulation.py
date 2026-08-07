import os
import json
from AreaModel import AreaModel
from RobotModels.UAVModel import UAVModel
import numpy as np
from collections import deque
from Record import RecordTime, RecordRedundancy
import math

class Simulation():
    def __init__(self):
        print("Create Simulation")
        self.GetParams()
        self.CalculateTotalLinkBudget()

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


    # Calculate total dBm for communication between robots
    def CalculateTotalLinkBudget(self):
        comms_params = self.UAVParams["Communications"]

        # Get params used to model wifi communication
        transmit_power = comms_params["TransmitPower"]
        receiver_sensitivity = comms_params["ReceiverSensitivity"]
        antennae_gains = comms_params["AntennaeGains"]
        interference_margin = comms_params["InterferenceMargin"]

        # Calculate if the total communication budget is bigger than the amount needed to communicate
        self.total_link_budget = transmit_power + antennae_gains - receiver_sensitivity - interference_margin


    # Step the robot one step on the grid
    def StepRobots(self):
        for robot in self.UAVs:
            robot.robot_next_step(self.startRobotIDs, self.time_step, self.area, self.time_step, self.recharge_point)

        # time.sleep(self.time_step)
        self.time_elapsed += self.time_step
        self.ShareRobotData()


    # Share the robot data between UAVs
    def ShareRobotData(self):
        for uav in self.UAVs:
            uav.localUAVs = []
            for uav2 in self.UAVs:
                if uav == uav2 or not uav2.released or not uav.released:
                    continue
                
                if self.is_comms_modelled:
                    transmission_possible = self.Is_Transmission_Possible(uav, uav2)
                else:
                    transmission_possible = True

                if transmission_possible:
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
            self.startRobotIDs = d["StartRobotIDs"]
            self.time_step = d["TimeStep"]
            self.recharge_point = d["RechargePoint"]
            self.is_comms_modelled = d["IsCommsModelled"] == 1
            self.UAVParams = d["UAVParams"]

    
    # Calculate if transmission is possible between two UAVs
    def Is_Transmission_Possible(self, uav1: UAVModel, uav2: UAVModel):
        # Check params
        step = 0.1
        epsilon = 1e-9

        # Difference between uav1 and uav2
        dx = uav2.x_pos - uav1.x_pos
        dy = uav2.y_pos - uav1.y_pos
        total_dist = math.hypot(dx, dy)

        # Drones are at the exact same point
        if total_dist < 1e-6:
            return True

        # Distance from uav1 to check point
        step = 0.1  # Step resolution in grid units
        num_steps = int(math.ceil(total_dist / step))
        
        # Normalized direction vector per step
        step_x = (dx / total_dist) * step
        step_y = (dy / total_dist) * step
        x_pos = uav1.x_pos
        y_pos = uav1.y_pos

        # Moniter the amount of free space and wall space between the drones
        free_space = 0
        wall_space = 0

        # Check along line until reaching other uav2
        for i in range(num_steps):
            grid_x = int(round(x_pos + epsilon))
            grid_y = int(round(y_pos + epsilon))

            # Boundary check safeguard before array lookup
            if 0 <= grid_y < self.area.grid.shape[0] and 0 <= grid_x < self.area.grid.shape[1]:
                if self.area.grid[grid_y, grid_x] == 1:
                    wall_space += step
                else:
                    free_space += step
            else:
                # Out of bounds grid cell treated as free space (or wall depending on your sim rules)
                free_space += step

            # Advance along ray
            x_pos += step_x
            y_pos += step_y

        comms_params = self.UAVParams["Communications"]

        # Get params used to model wifi communication
        frequency = comms_params["Frequency"]
        concrete_loss = comms_params["ConcreteLoss"]
        
        total_required = self.ConcreteLoss(wall_space, concrete_loss) + self.CalculateFreeSpaceLoss(free_space, frequency)

        return self.total_link_budget > total_required


    # Calculate signal loss through the amount of wall
    def ConcreteLoss(self, wall_space, concrete_loss):
        return wall_space * concrete_loss

    # Calculate signal loss through the free space
    def CalculateFreeSpaceLoss(self, free_space_dist, frequency):
        if free_space_dist <= 0.001:
            return 0.0

        return 20 * math.log10(free_space_dist / 1000) + 20 * math.log10(frequency) + 32.45