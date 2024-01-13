from networktables import NetworkTables
import time
import json

class NTManager:
    def __init__(self, update_value_func, update_connection_func, address: str):
        self.rio_address = address

        self.update_value_func = update_value_func
        self.update_connection_func = update_connection_func

        NetworkTables.initialize(server = address)
        NetworkTables.addConnectionListener(self.connection_callback)

        self.odom_table = NetworkTables.getTable("odometry_tracking")
        self.odom_table.addEntryListener(self.value_change_callback)
        
    def value_change_callback(self, table: str, key: str, value, is_new: bool):
        self.update_value_func(key, value)
        

    def connection_callback(self, connected: bool, info):
        if connected:
            print(f"Connected to {self.rio_address}")

        else:
            print(f"Failed to connect to {self.rio_address}")
        self.update_connection_func(connected, self.rio_address)