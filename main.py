#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from collections import deque
from math import tan, radians
from numpy import Inf
from pygeodesy.sphericalTrigonometry import LatLon
from pathplanning import PathPlanning
from waypointsmap import WaypointMap
from drones import Drones
import matplotlib.pyplot as plt


CREATE_MAP = True
#CREATE_MAP = False

PLOT_DISTANCES_NB_TURNS = False

"""
Martinique
"""
start_point = LatLon(14.810431373395028, -61.176753253004485)
#start_point = LatLon(14.80431373395028, -61.175753253004485)
start_point.text = 'Start'

a = LatLon(14.801717517497558, -61.17830598375636)
b = LatLon(14.803184637915658, -61.177470922032136)
c = LatLon(14.804110982101069, -61.176259107347896)
d = LatLon(14.80476281073443, -61.175343978459765)
e = LatLon(14.804147551878703, -61.17414211429372)
f = LatLon(14.802389075700889, -61.175630772903205)
g = LatLon(14.801758424759862, -61.176496729696545)
points = deque([a, b, c, d, e, f, g])

FLIGHT_ALTITUDE = 100



# Choose sensor from json file
drones = Drones()
CAMERA_NUMBER = 0
try:
    CAMERA_NUMBER = int(input('Enter your camera number: '))
except ValueError:
    print("Not a number")

camera = drones.get_camera(CAMERA_NUMBER)

print(F'Selected camera is {camera}')

# parameters of the camera
fieldOfView = camera.camera_fieldofview
imageResolutionX = camera.camera_resolution_X
imageResolutionY = camera.camera_resolution_Y
aspectRatio = imageResolutionX / imageResolutionY

# width and height of the projected area
width = 2 * FLIGHT_ALTITUDE * tan(radians(fieldOfView / 2))
height = width / aspectRatio

distance_min = Inf
nb_turns = Inf
best_angle_distance = None
best_angle_nb_turns = None
best_path_nb_turns = None
best_path_distance = None

turns_array=[]
distances_array=[]

print('Computing best angle ')
ANIMATION = "|/-\\"
idx = 0
best_path_nb_turns = None

# find the best angles for nb_turns and total distance from start_point
RANGE = range(0, 180, 1)
for angle in RANGE:
    print(ANIMATION[idx % len(ANIMATION)], end="\r")
    idx += 1

    Path_generator = PathPlanning(points=points,  bearing=angle, lateral_footprint=width,
                                  longitudinal_footprint=height, start_point=start_point, percent_recouvrement_lat=0.6, percent_recouvrement_lon=0.80)

    Path_generator.extra_point.append(start_point)

    # Generate the path
    Path_generator.generate_path_normal_plus()

    tmp_length, tmp_nb_turns = Path_generator.compute_length_and_turns()
    turns_array.append(tmp_nb_turns)
    distances_array.append(tmp_length)

    if not best_path_nb_turns:
        best_path_nb_turns = Path_generator
    
    #print('Angle {} distance {} nb_turns {}'.format(angle,tmp_length,tmp_nb_turns))
    # minimum distance
    if tmp_length < distance_min:
        distance_min = tmp_length
        best_angle_distance = angle
        best_path_distance = Path_generator
    # minimum turns
    if tmp_nb_turns < nb_turns :
        nb_turns = tmp_nb_turns 
        best_angle_nb_turns = angle
        best_path_nb_turns = Path_generator
        #print(F'Best nb turns {nb_turns} avec angle {angle}')

    if tmp_nb_turns == nb_turns and tmp_length<best_path_nb_turns.total_distance:
        nb_turns = tmp_nb_turns 
        best_angle_nb_turns = angle
        best_path_nb_turns = Path_generator

    #if tmp_nb_turns > nb_turns :
    #    break

best_distance, best_distance_nb_turns = best_path_distance.compute_length_and_turns()
best_nb_turns_distance, best_nb_turns = best_path_nb_turns.compute_length_and_turns()

print('#######################')
print(F'Distance : Angle {best_angle_distance} distance {best_distance} nb_turns {best_distance_nb_turns}')
print(F'Nb Turns : Angle {best_angle_nb_turns} distance {best_nb_turns_distance} nb_turns {best_nb_turns}')
print('#######################')


if PLOT_DISTANCES_NB_TURNS:
    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('Angle en °')
    ax1.set_ylabel('nombre de virages', color=color)
    RANGE = range(0, len(turns_array), 1)
    ax1.plot(RANGE, turns_array, color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:blue'
    ax2.set_ylabel('distance en m', color=color)  # we already handled the x-label with ax1
    ax2.plot(RANGE, distances_array, color=color)
    ax2.tick_params(axis='y', labelcolor=color)
    ax2.grid(True)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    #plt.grid(True)
    plt.show()

# J'ai choisit le moins de virage, mais la plus petite distance est aussi pertinante...
Path_generator = best_path_nb_turns

################ Create the map ######
if CREATE_MAP:
    the_map = WaypointMap(start_point)

    # Mapping area to the map
    the_map.add_polygon(points=points, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

    # Waypoints to the map
    for wp in Path_generator.waypoint_list:
        the_map.add_waypoint(wp, direction=False,
                             footprint=False, footprint_markers=False)

    # Add
    the_map.add_colored_waypoint_path(Path_generator.waypoint_list)

    # Extra points (for debug) to the map
    for extra in Path_generator.extra_point:
        the_map.add_extra(extra, text=extra.text, misc=extra.misc)
        #print('extra text '+extra.text + '\t\tExtra point\t' + str(extra.lat)+ '\t'+str(extra.lon))

    # Export the map
    the_map.export_to_file('best_angle_nb_turns')


# wp_list is the list of the waypoints
wp_list = Path_generator.export_to_list()
print(wp_list)
print("######################################")
wp_list = Path_generator.export_to_paired_wp()
print(wp_list)
