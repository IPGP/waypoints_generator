#!/usr/bin/python
# -*- coding:utf-8 -*-
from pygeodesy.sphericalTrigonometry import LatLon
from pathplanning import PathPlanning
from waypointsmap import WaypointMap
from collections import deque
from math import tan,radians
from drones import Drones
import sys

"""
Martinique
"""
start_point = LatLon(14.810431373395028, -61.176753253004485)
start_point.text = 'Start'

a = LatLon(14.801717517497558, -61.17830598375636)
b= LatLon(14.803184637915658, -61.177470922032136)
c= LatLon(14.804110982101069, -61.176259107347896)
d= LatLon(14.80476281073443, -61.175343978459765)
e= LatLon(14.804147551878703, -61.17414211429372)
f= LatLon(14.802389075700889, -61.175630772903205)
g= LatLon(14.801758424759862, -61.176496729696545)
points = deque([a, b, c, d, e,f,g])

flight_altitude = 20



## Choose sensor from json file
drones=Drones()
camera_number = 0
try:
    camera_number = int(input('Enter your camera number: '))
except ValueError:
    print("Not a number")

camera = drones.get_camera(camera_number)

print('Selected camera is {}'.format(camera))


# parameters of the camera
fieldOfView = camera.camera_fieldofview
imageResolutionX = camera.camera_resolution_X
imageResolutionY = camera.camera_resolution_Y
aspectRatio = imageResolutionX / imageResolutionY;

# width and height of the projected area
width = 2 * flight_altitude * tan(radians(fieldOfView / 2))
height = width / aspectRatio

print('Camera ')
Path_generator= PathPlanning(points=points,  bearing = 140,lateral_footprint=width,
                                longitudinal_footprint=height, start_point=start_point, percent_recouvrement_lat=0.6, percent_recouvrement_lon=0.80)

Path_generator.extra_point.append(start_point)

# Gerenate the path
Path_generator.generate_path("normal_plus")

## Create the map
the_map = WaypointMap(start_point)

# Mapping area to the map
the_map.add_polygon(points=points, color='#ff7800', fill=True,
                    fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")


# Waypoints to the map
for wp in Path_generator.waypoint_list:
    the_map.add_waypoint(wp, direction=False, footprint=False,footprint_markers=False)

# Add
the_map.add_colored_waypoint_path(Path_generator.waypoint_list)

# Extra points (for debug) to the map
for extra in Path_generator.extra_point:
    the_map.add_extra(extra, text=extra.text)
    #print('extra text '+extra.text + '\t\tExtra point\t' + str(extra.lat)+ '\t'+str(extra.lon))

# Export the map
the_map.export_to_file('normal_plus')

# wp_list is the list of the waypoints
wp_list =Path_generator.export_to_list()
print(wp_list)
print("######################################2
")
wp_list =Path_generator.export_to_paired_wp()
print(wp_list)
