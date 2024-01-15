import csv
import json
import math
from networktables import NetworkTables
import time
import argparse

parser = argparse.ArgumentParser(description = "script to gather and record vision measurement over networktables")
parser.add_argument("pos")

args = parser.parse_args()

measurement_duration = 5

POSITIONS = [
    (1.41, 0.14),
    (2.43, 0.21),
    (3.34, 0.23),
    (4.3, 0.29),
    (5.27, 0.36),
    (1.61, 1.15),
    (2.44, 1.07),
    (3.39, 1.06),
    (4.38, 1.1),
    (5.34, 1.13),
    (6.33, 1.07),
    (3.21, 1.73),
    (4.14, 2.28),
    (4.15, 1.77),
    (5.1, 1.71)
],

class MeasurementRecorder:
    def __init__(self):
        self.pos = args.pos

        self.RIO_ADDRESS = "10.44.99.2"

        NetworkTables.initialize(server = self.RIO_ADDRESS)
        NetworkTables.addConnectionListener(self.connection_callback)
        self.measurement_table = NetworkTables.getTable("measurements")
        self.measurement_table.addEntryListener(self.value_change_callback)

    def connection_callback(self, connected: bool, info):
        if connected:
            print(f"Connected to {self.RIO_ADDRESS}")
        else:
            print(f"Unable to connect to {self.RIO_ADDRESS}")

    def value_change_callback(self, table: str, key: str, value, isNew: bool):
        print(f"{key}: {value}")
        if key == "tag":
            with open(f"data/tag_poses_{self.pos}.csv", "a") as f:
                writer = csv.writer(f)
                writer.writerow(value)
        elif key == "triangulation":
            with open(f"data/triangulation_poses_{self.pos}.csv", "a") as f:
                writer = csv.writer(f)
                writer.writerow(value)

print("running")

mr = MeasurementRecorder()

start = time.time()
while time.time() - start < measurement_duration:
    pass
print("done")
exit()