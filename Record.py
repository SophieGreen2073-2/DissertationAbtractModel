import numpy as np
import csv

class RecordTime():
    def record_time_elapsed(self, num_robots, time_elapsed):
        with open('dissertation_time_record.csv', 'a') as f:
            new_data = np.hstack((num_robots, time_elapsed))
            np.savetxt(f, [new_data], delimiter=',', fmt='%.6f')


class RecordRedundancy():
    def record_overlap(self, overlap_area, numUAVs):
        with open('dissertation_redundancy_record.csv', 'a') as f:
            new_data = np.hstack((overlap_area, numUAVs))
            np.savetxt(f, [new_data], delimiter=',', fmt='%.6f')
        