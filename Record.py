import numpy as np
import csv

class RecordTime():
    def record_time_elapsed(self, num_robots, time_elapsed, uav_params, algorithm, comms, drone):
        comms_string = "comms" if comms else "no_comms"

        with open(f'NewSavedData/dissertation_time_record_{algorithm}_{comms_string}_{drone}_{num_robots}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            
            # 1. Start with your base variables safely converted
            row = [int(num_robots), f"{time_elapsed:.6f}"]
            
            # 2. Extract and iterate through the dict values, skipping the keys
            # .values() gives you the actual parameters (e.g., 2.5, "Mavic", True)
            for param in uav_params.values():
                if isinstance(param, float):
                    row.append(f"{param:.6f}") # Safely format floats to 6 decimal places
                elif isinstance(param, (int, bool)):
                    row.append(int(param))     # Write integers/booleans cleanly without decimal drift
                else:
                    row.append(str(param))     # Write text, strings, or labels exactly as they are
            
            # 3. Append the mixed data row directly to the CSV
            writer.writerow(row)


class RecordRedundancy():
    def record_overlap(self, overlap_area, numUAVs, uav_params, algorithm, comms, drone):
        comms_string = "comms" if comms else "no_comms"

        with open(f'NewSavedData/dissertation_redundancy_record_{algorithm}_{comms_string}_{drone}_{numUAVs}.csv', 'a') as f:
            writer = csv.writer(f)
            
            # 1. Start with your base variables safely converted
            row = [int(numUAVs)]

            for val in overlap_area.ravel():
                row.append(f"{float(val):.6f}")
            
            # 2. Extract and iterate through the dict values, skipping the keys
            # .values() gives you the actual parameters (e.g., 2.5, "Mavic", True)
            for param in uav_params.values():
                if isinstance(param, float):
                    row.append(f"{param:.6f}") # Safely format floats to 6 decimal places
                elif isinstance(param, (int, bool)):
                    row.append(int(param))     # Write integers/booleans cleanly without decimal drift
                else:
                    row.append(str(param))     # Write text, strings, or labels exactly as they are
            
            # 3. Append the mixed data row directly to the CSV
            writer.writerow(row)


class RecordScannedGrid():
    def save_path_taken(self, uavs, uav_params, num_UAVs, algorithm, comms, drone):
        comms_string = "comms" if comms else "no_comms"

        with open(f'NewSavedData/dissertation_path_taken_record_{algorithm}_{comms_string}_{drone}_{num_UAVs}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            
            for i, ugv in enumerate(uavs):
                row = [int(i)]
                
                # Safely flatten list of tuples/lists using pure Python comprehension
                if len(ugv.path_taken) > 0:
                    flattened_grid = [val for item in ugv.path_taken for val in item]
                    for val in flattened_grid:
                        row.append(f"{float(val):.6f}")
                
                for param in uav_params.values():
                    if isinstance(param, float):
                        row.append(f"{param:.6f}")
                    elif isinstance(param, (int, bool)):
                        row.append(int(param))
                    else:
                        row.append(str(param))
                
                writer.writerow(row)

    def save_paths_planned(self, uavs, uav_params, num_UAVs, algorithm, comms, drone):
        comms_string = "comms" if comms else "no_comms"

        with open(f'NewSavedData/dissertation_paths_planned_record_{algorithm}_{comms_string}_{drone}_{num_UAVs}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            
            for i, ugv in enumerate(uavs):
                row = [int(i)]
                
                # Robust helper to recursively flatten any nested lists or tuples
                def flatten_recursive(data):
                    for item in data:
                        if isinstance(item, (list, tuple)):
                            yield from flatten_recursive(item)
                        else:
                            yield item

                if len(ugv.paths_planned) > 0:
                    for val in flatten_recursive(ugv.paths_planned):
                        try:
                            row.append(f"{float(val):.6f}")
                        except (ValueError, TypeError):
                            # Fallback if it's a non-numeric string or object
                            row.append(str(val))
                
                for param in uav_params.values():
                    if isinstance(param, float):
                        row.append(f"{param:.6f}")
                    elif isinstance(param, (int, bool)):
                        row.append(int(param))
                    else:
                        row.append(str(param))
                
                writer.writerow(row)
                
    def save_exploration_timing(self, uavs, uav_params, num_UAVs, algorithm, comms, drone):
        comms_string = "comms" if comms else "no_comms"

        with open(f'NewSavedData/dissertation_exploration_timing_record_{algorithm}_{comms_string}_{drone}_{num_UAVs}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            
            for i, ugv in enumerate(uavs):
                row = [int(i)]
                
                # Safely handle exploration_timing whether it's a float, a list of floats, or nested lists
                timing_data = ugv.algorithm_timing
                if timing_data is not None:
                    if isinstance(timing_data, (float, int, np.number)):
                        # If it's a single number, wrap it in a list
                        flattened_grid = [float(timing_data)]
                    elif isinstance(timing_data, (list, tuple)):
                        # Safely flatten nested or flat structures
                        flattened_grid = []
                        for item in timing_data:
                            if isinstance(item, (list, tuple, np.ndarray)):
                                flattened_grid.extend([val for val in item])
                            else:
                                flattened_grid.append(item)
                    else:
                        flattened_grid = []

                    for val in flattened_grid:
                        try:
                            row.append(f"{float(val):.6f}")
                        except (ValueError, TypeError):
                            row.append(str(val))
                
                for param in uav_params.values():
                    if isinstance(param, float):
                        row.append(f"{param:.6f}")
                    elif isinstance(param, (int, bool)):
                        row.append(int(param))
                    else:
                        row.append(str(param))
                
                writer.writerow(row)