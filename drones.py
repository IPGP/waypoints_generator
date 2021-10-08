#!/usr/bin/python
# -*- coding:utf-8 -*-
import json
import sys
# Json valider
# https://jsonlint.com/

class Camera:
    "Camera class"
    def __init__(self,json_drone,json_camera):
        """Camera data """
        self.drone_brand = json_drone['brand']
        self.drone_name = json_drone['name']
        self.camera_name = json_camera['camera']
        self.camera_resolution_X = json_camera['resolution_X']
        self.camera_resolution_Y = json_camera['resolution_Y']
        self.camera_fieldOfView = json_camera['fieldOfView']
        self.camera_x_sensor_size = json_camera['x_sensor_size']
        self.camera_y_sensor_size = json_camera['y_sensor_size']

    def __str__(self):
        camera_string = self.drone_brand
        camera_string += ' - '+self.drone_name
        camera_string += ' - '+self.camera_name
        camera_string += ' - '+str(self.camera_resolution_X)
        camera_string += ' - '+str(self.camera_resolution_Y)
        camera_string += ' - '+str(self.camera_fieldOfView)
        camera_string += ' - '+str(self.camera_x_sensor_size)
        camera_string += ' - '+str(self.camera_y_sensor_size)
        return camera_string
        
class Drones:
    "Drones class"

    def __init__(self,json_file='drones.json'):
        """Drones get data from json file """

        # Read JSON file
        with open(json_file) as data_file:
            data_loaded = json.load(data_file)

        #print(json.dumps(data_loaded, indent=2, sort_keys=False))
        for drone in data_loaded['Drones']:
            for cam in drone['Cameras']:
                c=Camera(drone,cam)
                print(c)



def main(args):
    """la fonction main"""
    a=Drones()
if __name__ == '__main__':
    main(sys.argv)
