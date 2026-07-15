import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import os

class AreaModel:
    def __init__(self):
        print("Create model")

        self.cmap = colors.ListedColormap([
            # --- YOUR ORIGINAL 5 COLORS ---
            '#FFFFFF',  # 0: Empty Space / Unexplored (White)
            '#000000',  # 1: Walls / Obstacles (Black)
            '#008000',  # 2: Discovered Areas / Frontiers (Green)
            '#FF0000',  # 3: Targets / Goals (Red)
            '#0000FF',  # 4: Main UAV (Blue)

            # --- ADDITIONAL HIGH-CONTRAST COLORS (5 to 19) ---
            '#FF8C00',  # 5: Dark Orange (e.g., UAV 2)
            '#800080',  # 6: Purple (e.g., UAV 3)
            '#00FFFF',  # 7: Cyan (e.g., Sensor Coverage / FOV)
            '#FF00FF',  # 8: Magenta (e.g., Shared Map / Ad-hoc Link)
            '#FFD700',  # 9: Gold / Yellow (e.g., Doors / Portals)
            '#4B0082',  # 10: Indigo
            '#7FFF00',  # 11: Chartreuse / Lime
            '#FF1493',  # 12: Deep Pink
            '#1E90FF',  # 13: Dodger Blue
            '#A0522D',  # 14: Sienna Brown (e.g., Static Floor Obstacles)
            '#00FF00',  # 15: Bright Green
            '#8B0000',  # 16: Dark Red
            '#008080',  # 17: Teal
            '#708090',  # 18: Slate Gray (e.g., GPS-denied shadow regions)
            '#FF6347'   # 19: Tomato Red
        ])
      
    def BuildModel(self, numUAVs):
        current_dir = os.path.dirname(os.path.abspath(__file__))
    
        # Securely join that directory path with your JSON filename
        json_path = os.path.join(current_dir, 'AreaLayout.JSON')
        with open(json_path) as f:
            d = json.load(f)

            self.height = int(d["fullarea"]["height"])
            self.width = int(d["fullarea"]["width"])

            self.grid = np.zeros((self.height, self.width))

            walls = d["walls"]
            for wall in walls:
                x_start, x_end = min(wall["start"][0], wall["end"][0]), max(wall["start"][0], wall["end"][0])
                y_start, y_end = min(wall["start"][1], wall["end"][1]), max(wall["start"][1], wall["end"][1])

                # Horizontal wall
                if (y_start == y_end):
                    self.grid[y_start, x_start:x_end + 1] = 1
                # Vertical wall
                else:
                    self.grid[y_start:y_end + 1, x_start:x_end + 1] = 1

            doors = d["doors"]
            for door in doors:
                self.grid[door[1], door[0]] = 0

            targets = d["targets"]
            for target in targets:
                pos_x, pos_y = target["position"][0], target["position"][1]
                self.grid[pos_y, pos_x] = 2

            floor_obstacles = d["floor_obstacles"]
            for floor_obstacle in floor_obstacles:
                x_start, x_end = min(floor_obstacle["start"][0], floor_obstacle["end"][0]), max(floor_obstacle["start"][0], floor_obstacle["end"][0])
                y_start, y_end = min(floor_obstacle["start"][1], floor_obstacle["end"][1]), max(floor_obstacle["start"][1], floor_obstacle["end"][1])

                self.grid[y_start:y_end, x_start:x_end] = 3

        self.overlap_area = np.zeros((self.height, self.width, numUAVs))

        

    def DisplayGrid(self):
        plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(12,8))
        self.im = self.ax.imshow(self.grid, cmap=self.cmap, interpolation='nearest', origin='upper', vmin=0, vmax=19)

        self.ax.set_xticks(np.arange(-0.5, self.width, 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, self.height, 1), minor=True)
        self.ax.grid(which="minor", color="#D3D3D3", linestyle="-", linewidth=0.5)

        # Show the window
        plt.pause(0.1)

    def DisplayStaticGrid(self):
        self.fig, self.ax = plt.subplots(figsize=(12,8))
        self.im = self.ax.imshow(self.grid, cmap=self.cmap, interpolation='nearest', origin='upper', vmin=0, vmax=19)

        self.ax.set_xticks(np.arange(-0.5, self.width, 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, self.height, 1), minor=True)
        self.ax.grid(which="minor", color="#D3D3D3", linestyle="-", linewidth=0.5)

        # Show the window
        plt.show()

    def UpdateGrid(self):
        if self.im is not None:
            self.im.set_data(self.grid)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)

    def area_scan(self, robot_id, x, y):
        if self.grid[y, x] == 0: 
            self.grid[y, x] = robot_id
                