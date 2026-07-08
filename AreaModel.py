import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import os

class AreaModel:
    def __init__(self):
        print("Create model")

    def BuildModel(self):
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

    def DisplayGrid(self):
        plt.ion()

        cmap = colors.ListedColormap(['#FFFFFF', '#000000', '#008000', '#FF0000', '#0000FF'])
        self.fig, self.ax = plt.subplots(figsize=(12,8))
        self.im = self.ax.imshow(self.grid, cmap=cmap, interpolation='nearest', origin='upper')

        self.ax.set_xticks(np.arange(-0.5, self.width, 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, self.height, 1), minor=True)
        self.ax.grid(which="minor", color="#D3D3D3", linestyle="-", linewidth=0.5)

        # Show the window
        # plt.show()
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        plt.pause(0.1)

    def DisplayStaticGrid(self):
        cmap = colors.ListedColormap(['#FFFFFF', '#000000', '#008000', '#FF0000', '#0000FF'])
        self.fig, self.ax = plt.subplots(figsize=(12,8))
        self.im = self.ax.imshow(self.grid, cmap=cmap, interpolation='nearest', origin='upper')

        self.ax.set_xticks(np.arange(-0.5, self.width, 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, self.height, 1), minor=True)
        self.ax.grid(which="minor", color="#D3D3D3", linestyle="-", linewidth=0.5)

        # Show the window
        plt.show()

    def UpdateGrid(self):
        if self.im is not None:
            self.im.set_data(self.grid)
            self.im.set_clim(vmin=self.grid.min(), vmax=self.grid.max())
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)

    def area_scan(self, robot_id, x, y):
        if self.grid[y, x] == 0: self.grid[y, x] = robot_id
                