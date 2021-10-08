#!/usr/bin/python
# -*- coding:utf-8 -*-
import json
import sys
# Json valider
# https://jsonlint.com/

class Camera:
    "Camera class"
    def __init__(self,i,json_drone,json_camera):
        """Camera data """
        self.camera_number = i
        self.drone_brand = json_drone['brand']
        self.drone_name = json_drone['name']
        self.camera_name = json_camera['camera']
        self.camera_resolution_X = json_camera['resolution_X']
        self.camera_resolution_Y = json_camera['resolution_Y']
        self.camera_fieldofview = json_camera['fieldOfView']
        self.camera_x_sensor_size = json_camera['x_sensor_size']
        self.camera_y_sensor_size = json_camera['y_sensor_size']


    def __str__(self):
        camera_string = '{:^6} {:^22s} \t {:^20s}\t'.format(self.camera_number,self.drone_brand+' '+self.drone_name,self.camera_name)
        camera_string += '{:^9}{:^9}\t'.format(self.camera_resolution_X,self.camera_resolution_Y)
        camera_string += '{:^13}\t'.format(self.camera_fieldofview)
        camera_string += '{:^6} {:^6}'.format(self.camera_x_sensor_size,self.camera_y_sensor_size)
        return camera_string
        
class Drones:
    "Drones class"

    def __init__(self,json_file='drones.json'):
        """Drones get data from json file """
        self.camera_list=[]
        # Read JSON file
        with open(json_file) as data_file:
            data_loaded = json.load(data_file)
        header = '{:^6s} {:^22s} \t {:^20s}\t'.format('Number','Drone','Camera')
        header += '{:^18s}\t'.format('Sensor resolution')
        header += '{:^13s}\t'.format('Field of view')
        header += '{:^12s}'.format('Sensor size')
        print(header)
        i = 0
        for drone in data_loaded['Drones']:
            for cam in drone['Cameras']:
                camdrone=Camera(i,drone,cam)
                self.camera_list.append(camdrone)
                print(camdrone)
                i +=1

    def get_camera(self,camera_number):
        """return camera with camera_number index
        """
        return self.camera_list[camera_number]

def main(args):
    """la fonction main"""
    a=Drones()
if __name__ == '__main__':
    main(sys.argv)
