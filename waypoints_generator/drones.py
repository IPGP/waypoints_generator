#!/usr/bin/python
# -*- coding:utf-8 -*-
import json
import sys
from utils import bfg, bg
# Json valider
# https://jsonlint.com/


class Camera:
    "Camera class"

    def __init__(self, json_drone, json_camera):
        """Camera data """
        self.drone_brand = json_drone['brand']
        self.drone_name = json_drone['name']
        self.camera_id = json_camera['id']
        self.camera_name = json_camera['camera']
        self.camera_resolution_x = json_camera['resolution_X']
        self.camera_resolution_y = json_camera['resolution_Y']
        self.camera_focal = json_camera['focal']
        self.camera_x_sensor_size = json_camera['x_sensor_size']
        self.camera_y_sensor_size = json_camera['y_sensor_size']

    def __str__(self):

        camera_string = F"{self.camera_id :^12s}|{self.drone_brand+' '+self.drone_name :^22s}|{self.camera_name :^20s}|"
        camera_string += F"{self.camera_resolution_x:>9} x {self.camera_resolution_y:<8}|"
        camera_string += F"{self.camera_focal:^13}|"
        camera_string += F"{self.camera_x_sensor_size:>5} x {self.camera_y_sensor_size:<6}|"
        return camera_string

    def header(self):

        # Header
        header = f"{'Id':^12s}|{'Drone':^22s}|{'Camera':^20s}|"
        header += f"{'Sensor resolution px':^17s}|"
        header += f"{'Focal mm':^13s}|"
        header += f"{'Sensor size mm':^13s}|"
        return header

class Drones:
    "Drones class"

    def __init__(self, json_file='drones.json'):
        """Drones get data from json file """
        self.camera_dict = {}
        # Read JSON file
        with open(json_file) as data_file:
            self.data_loaded = json.load(data_file)

        for drone in self.data_loaded['Drones']:
            for cam in drone['Cameras']:
                camdrone = Camera(drone, cam)
                self.camera_dict[camdrone.camera_id] = Camera(drone, cam)

    def print(self):
        """Custom print() function."""

        # Header
        header = f"{'Id':^12s}|{'Drone':^22s}|{'Camera':^20s}|"
        header += f"{'Sensor resolution px':^17s}|"
        header += f"{'Focal mm':^13s}|"
        header += f"{'Sensor size mm':^13s}|"
        print(bfg(header, 93, 11))

        # Cameras with alternate colors
#        for i,camdrone in enumerate(self.camera_dict.items()):
        for i, (key, camdrone) in enumerate(self.camera_dict.items()):
            if i % 2 == 0:
                print(bg(camdrone.__str__(), 28))
            else:
                print(bg(camdrone.__str__(), 19))

    def get_camera_by_id(self, camera_id):
        """return camera with camera_number index
        """
        cam = self.camera_dict.get(camera_id)
        if cam:
            return cam
        else:
            print(F"camera_id {bg(camera_id,9)} is not in the database. Available camera are")
            self.print()
            sys.exit(-1)


def main():
    """la fonction main"""
    drones_db = Drones()
    drones_db.print()


if __name__ == '__main__':
    main()
