import time
import numpy as np

class RecordTime():
    def StartRecord(self):
        self.start = time.time()
    
    def EndRecord(self):
        self.length = time.time() - self.start


class RecordRedundancy():
    def MeasureOverlap(self, overlap_area, numUAVs):
        scans_per_robot_total = np.zeros(numUAVs)        
        for row in overlap_area:
            for cell in row:
                for robot_id in len(cell):
                    scans_per_robot_total[robot_id] += cell[robot_id]
        
        