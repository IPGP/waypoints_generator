#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from math import tan, radians
import os
import sys
from collections import deque
from itertools import cycle
import time
from argparse import ArgumentParser,ArgumentDefaultsHelpFormatter
import configparser
from ast import literal_eval
from unicodedata import name
from numpy import Inf
from pygeodesy.sphericalTrigonometry import LatLon
from pygeodesy.sphericalNvector  import intersection, LatLon as LatLonS
from pygeodesy.points import isclockwise, isconvex, centroidOf
from utils import getAnglelatlon
from waypoint import WayPoint
from waypointsmap import WaypointMap
from pyproj import Transformer
from PIL import Image as ImagePil
import numpy as np
from dict2djikml import dict2djikml 
from drone_orientation.classes.droneorientation import DroneOri
from drones import Camera, Drones
from utils import bfg, bg, fg

# https://nbviewer.jupyter.org/github/python-visualization/folium/blob/master/examples/Rotate_icon.ipynb
# rotation des icones
# Faire des schémas
# https://www.math10.com/en/geometry/geogebra/fullscreen.html
# emoji list https://unicode.org/emoji/charts/full-emoji-list.html#23f3

DEBUG = False
#DEBUG = True


class PathPlanning:
    "PathPlanning class"

    def __init__(self, points,  bearing=0, lateral_footprint=0, longitudinal_footprint=0,
     percent_recouvrement_lat=0., percent_recouvrement_lon=0., orientation=None, start_point=None):
        """Pathplanning generates waypoints """
        if not isconvex(tuple(points)):
            print("points must form a convex shape")
            sys.exit(-1)
        self.start_time = time.time_ns()


        self.points = points
        ## All the points in LatLonS for reference
        self.points_LatLonS = []
        for point in self.points:
            self.points_LatLonS.append(LatLonS(point.lat, point.lon))

        self.CENTROID_LATLON_S = centroidOf(self.points, LatLon=LatLonS)
        self.CENTROID_LATLON = centroidOf(self.points, LatLon=LatLon)

        self.nb_points = len(self.points)
        self.bearing = bearing
        if DEBUG: print('Les points de ce pathplanning sont {}'.format(self.points))
        self.waypoint_list = []
        self.paired_waypoints_list = []
        self.extra_point = []

        self.distances = []
        self.bearings = []
        if isclockwise(points):
            self.othogonal_increment = 90
            self.isclockwise = True
        else:
            self.othogonal_increment = -90
            self.isclockwise = False

        self.orientation = orientation
        self.start_point = start_point
        self.lateral_footprint = lateral_footprint  # en mètres
        self.longitudinal_footprint = longitudinal_footprint  # en mètres
        self.recouvrement_lat = percent_recouvrement_lat  # pourcentage
        self.recouvrement_lon = percent_recouvrement_lon  # pourcentage
        self.increment_lon = self.longitudinal_footprint * \
            (1-percent_recouvrement_lon)
        self.increment_lat = self.lateral_footprint * \
            (1-percent_recouvrement_lat)
        self.compute_distances_and_bearings(self.points)

    def create_segments(self):
        # Create all the segments of the area
        self.area_segments = []
        i = 0
        point_cycle = cycle(self.points)
        next_point = next(point_cycle)
        while i < self.nb_points:
            this_point, next_point = next_point, next(point_cycle)
            self.area_segments.append([LatLonS(this_point.lat, this_point.lon), LatLonS(next_point.lat, next_point.lon)])
            i += 1

        #print(F"Init\t{time.time_ns()-self.start_time}")

    def new_bearing(self, new_bearing):
        """
        change the bearing of this pathplanning
        """
        self.bearing = new_bearing
        self.waypoint_list = []
        self.paired_waypoints_list = []
        self.extra_point = []

    def compute_distances_and_bearings(self, input_points):
        """ compute distances between all points"""
        i = 0
        point_cycle = cycle(input_points)
        next_point = next(point_cycle)
        preceding_point = input_points[-1]
        while i < len(input_points):
            this_point, next_point = next_point, next(point_cycle)
            this_point.angle = getAnglelatlon(preceding_point, this_point, next_point)
            this_point.bearing = this_point.compassAngleTo(next_point)
            this_point.l = this_point.distanceTo(next_point)
            preceding_point = this_point
            i += 1

    def generate_path_normal_plus(self):
        """ Generate the path """

        # Find the farthest-best starting point for self.bearing
        # start_point = ref_point

        ref_point = self.points[0]
        distance_max_to_centroid = 0
        projection_point = None

        ### 
        for point in self.points_LatLonS:
            intersection_point = intersection(point, self.bearing, self.CENTROID_LATLON_S, self.bearing+90)
            distance_to_centroid = intersection_point.distanceTo(self.CENTROID_LATLON_S)

            if distance_to_centroid > 10000:
                intersection_point = intersection_point.antipode()
                distance_to_centroid = intersection_point.distanceTo(self.CENTROID_LATLON_S)

            if distance_to_centroid > distance_max_to_centroid:
                distance_max_to_centroid = distance_to_centroid
                ref_point = point
                projection_point = LatLon(intersection_point.lat, intersection_point.lon)

        ref_point.text = 'ref point'
        #print(F"Pouf\t{time.time_ns()-self.start_time}")

        # rotate the summits to have self.points[0] == ref_point
        while self.points[0].lat != ref_point.lat and self.points[0].lon != ref_point.lon :
            self.points.rotate()
        # now we can find all the segments
        self.create_segments()

        # find direction from ref_point to centroid
        direction_to_centroid = projection_point.compassAngleTo(self.CENTROID_LATLON)

        # If we start from self.farest_summit
        intersection_list = []
        intersection_exist = True

        i = 1

        # We search until we do not have intersections points between
        # parralle lines to bearing with increasing distance from ref_point
        while intersection_exist :
            new_point = ref_point.destination(i*self.increment_lat, direction_to_centroid)
            new_point_LatLonS = LatLonS(new_point.lat, new_point.lon)
            new_point_LatLonS.text="nw_pt "+str(i)
            #self.extra_point.append(new_point_LatLonS)

            intersection_exist = False
            nb_intersection = 0
            # find intersections points
            for segment in self.area_segments:

                #Compute intersection
                intersection_point_A = intersection(new_point_LatLonS, self.bearing, segment[0], segment[1])
                intersection_point_B = intersection(new_point_LatLonS, self.bearing+180, segment[0], segment[1])

                # the intersection point should be in the segment
                if intersection_point_A and intersection_point_A.iswithin(segment[0], segment[1]):
                    intersection_list.append(intersection_point_A)
                    intersection_exist = True
                    nb_intersection +=1
                    #print(F'{intersection_point_A.lat}\t{intersection_point_A.lon}')
                elif intersection_point_B and intersection_point_B.iswithin(segment[0], segment[1]):
                    intersection_list.append(intersection_point_B)
                    intersection_exist = True
                    nb_intersection +=1
                    #print(F'\t\t{intersection_point_B.lat}\t{intersection_point_B.lon}')

                # If 2 intersections have been found, we break this loop
                if nb_intersection >= 2:
                    break

            i += 1
        #print(F"Paf\t{time.time_ns()-self.start_time}")

        # we need to reorder the pairs of intersection point
        flip_left_right = True

        #On peut ajouter le point de référence, mais iln'est pas sur un profil et ne sera donc pas utilisé
        # the summit will be the first WP
        #self.waypoint_list.append(WayPoint(
        #    ref_point, self.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))

        #On peut ajouter le point de référence, mais iln'est pas sur un profil et ne sera donc pas utilisé
        #self.paired_waypoints_list.append([[ref_point.lat, ref_point.lon], []])

        for i in range(0, len(intersection_list)-1, 2):
            if flip_left_right:
                self.waypoint_list.append(WayPoint(
                    intersection_list[i], self.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.waypoint_list.append(WayPoint(
                    intersection_list[i+1], self.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.paired_waypoints_list.append([[intersection_list[i].lat, intersection_list[i].lon], [
                                                  intersection_list[i+1].lat, intersection_list[i+1].lon]])
            else:
                self.waypoint_list.append(WayPoint(
                    intersection_list[i+1], self.bearing+180, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.waypoint_list.append(WayPoint(
                    intersection_list[i], self.bearing+180, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))
                self.paired_waypoints_list.append([[intersection_list[i+1].lat, intersection_list[i+1].lon], [
                                                  intersection_list[i].lat, intersection_list[i].lon]])

            flip_left_right = not flip_left_right
            
        #print(F"Plouf\t{time.time_ns()-self.start_time}")

    def compute_length_and_turns(self):
        """ return the path length and turn numbers from and to the start point"""

        total_distance = 0.0
        nb_turns = 0

        # start point
        tmp_point = self.start_point

        for waypoint in self.waypoint_list:
            #print('type(waypoint) {} type(tmp_point) {}'.format(waypoint,type(tmp_point)))
            total_distance += tmp_point.distanceTo(
                LatLon(waypoint.point.lat, waypoint.point.lon))
            tmp_point = LatLon(waypoint.point.lat, waypoint.point.lon)
            nb_turns += 1

        total_distance += tmp_point.distanceTo(self.start_point)

      #  print('############################################################')
     #   print('{} Total distance is {}m with {} turns'.format(self.style, total_distance, nb_turns))
      #  print('############################################################')
        self.total_distance = total_distance
        self.nb_turns = nb_turns
        return total_distance, nb_turns

    def export_to_list(self):
        """ Export waypoints to a list"""
        wp_list = []
        for wp in self.waypoint_list:
            wp_list.append(wp.latlon())
        return wp_list

    def export_to_paired_wp(self):
        """ Export waypoints in pairs"""
        return self.paired_waypoints_list


def main():

    arg_parser = ArgumentParser(prog='Waypoints Generator',
                            description='Create waypoints from mnt with terrain awareness',
                            formatter_class=ArgumentDefaultsHelpFormatter)

    arg_parser.add_argument ("-c", "--config", dest='config_file', 
    help='the ini file',
    default='sapine.ini', type=str,required=True)
    args = arg_parser.parse_args()

    # parameters from INI file
    parser = configparser.ConfigParser()

    if args.config_file and os.path.isfile(args.config_file):
        parser.read(args.config_file)
    else :
        print(F'{args.config_file} does not exist or wrong path')
        sys.exit(-1)
    

    #### PROJECT
    # name
    project_name = parser.get("project","name")

    ### shape points
    points = deque()
    points_list = parser.get("project","points_list")
    for point in literal_eval(points_list):
        points.append(LatLon(point[0],point[1]))

    # start point
    takeoff_point = None
    takeoff_altitude = None
    
    a,b=literal_eval(parser.get("project","takeoff_point"))
    takeoff_point = LatLon(a,b)
    takeoff_point.text = 'Takeoff Point'
    if 'takeoff_altitude' in parser['project']:
        takeoff_altitude = float(parser.get("project","takeoff_altitude"))
    

    #### DRONE
    drone_speed = float(parser.get("drone_parameters","drone_speed"))
    onfinish=parser.get("drone_parameters","onfinish")
    ground_distance = float(parser.get("drone_parameters","ground_distance"))
    side_overlap=float(parser.get("drone_parameters","side_overlap"))
    front_overlap=float(parser.get("drone_parameters","front_overlap"))
    drone_azimuth=int(parser.get("drone_parameters","drone_azimuth"))
    id_camera=parser.get("drone_parameters","id_camera")
    drones_camera_db = Drones()
    camera = drones_camera_db.get_camera_by_id(id_camera)


    #### MNT
    dsm = parser.get("MNT","dsm")
    if not os.path.isfile(dsm):
        print(F'{dsm} does not exist or wrong path')
        sys.exit(-1)

    #shaded_dsm = parser.get("MNT","shaded_dsm")
    #if not os.path.isfile(shaded_dsm):
    #    print(F'{shaded_dsm} does not exist or wrong path')
    #    sys.exit(-1)

    tfw = parser.get("MNT","tfw")
    if not os.path.isfile(tfw):
        print(F'{tfw} does not exist or wrong path')
        sys.exit(-1)

    epsg_mnt = parser.get("MNT","epsg_mnt")

    if 'fixed_pitch' in parser['MNT']:
        fixed_pitch = int(parser.get("MNT","fixed_pitch"))
    else:
        fixed_pitch = None

    # a calculer a partir du dico JSON
    # Available data
    # camera.camera_name 
    # camera.camera_resolution_x 
    # camera.camera_resolution_y 
    # camera.camera_focal  
    # camera.camera_x_sensor_size 
    # camera.camera_y_sensor_size 

    
    print(fg('Selected \U0001f4f7 is ',87))
    print(camera.header())
    print(bfg(camera.__str__(),106,232))

    lateral_footprint = camera.camera_x_sensor_size*ground_distance/float(camera.camera_focal)  
    longitudinal_footprint = camera.camera_y_sensor_size*ground_distance/float(camera.camera_focal)

    print(fg(F'lateral_footprint: {lateral_footprint}m\tlongitudinal_footprint {longitudinal_footprint}m',14))
    print(fg(F'lateral_gsd: {lateral_footprint/camera.camera_resolution_x:.5f}m\tlongitudinal_gsd {longitudinal_footprint/camera.camera_resolution_y:.5f}m',14))
    
    Path_generator = PathPlanning(points=points,  bearing=drone_azimuth, lateral_footprint=lateral_footprint,
                                  longitudinal_footprint=longitudinal_footprint, start_point=takeoff_point, percent_recouvrement_lat=side_overlap, percent_recouvrement_lon=front_overlap)

    Path_generator.extra_point.append(takeoff_point)
    Path_generator.generate_path_normal_plus()
 
 
#    for paire in Path_generator.export_to_paired_wp():
#        print(paire)

    ################### Profils ##########################

    # conversion des coordonnes https://pyproj4.github.io/pyproj/stable/gotchas.html#upgrading-to-pyproj-2-from-pyproj-1
    epsg_in = "epsg:4126"
    coordonates_transformer = Transformer.from_crs(epsg_in, epsg_mnt)
    reverse_coordonates_transformer = Transformer.from_crs(epsg_mnt, epsg_in)
    final_waypoint_dict = []
   
    # i sert à nommer les numéros de profils
    for i, paire in enumerate(Path_generator.export_to_paired_wp()):
        
        # test pour ne garder que les paires de points
        if paire[0] and paire [1]:
            #print(paire)
            a_east,a_north = coordonates_transformer.transform(paire[0][0],paire[0][1])
            b_east, b_north = coordonates_transformer.transform(paire[1][0],paire[1][1])
            #print(F'\nCoordonnées converties => DroneOri a \t{a_north}\t{a_east}')
            #print(F'Coordonnées converties => DroneOri b \t{b_north}\t{b_east}\n')
            #print(f'a_east {a_east},a_north {a_north} b_east {b_east},b_north {b_north} ')
            print("\U000023f3 Computing "+project_name+'_'+str(i)+ " profile")

            if takeoff_altitude:
                prof1 = DroneOri(
                name=project_name+'_'+str(i), dsm=dsm, tfw=tfw,
                a_east=a_east, a_north=a_north, b_east=b_east, b_north=b_north, 
                h=ground_distance, sensor_size=(float(camera.camera_x_sensor_size) ,float(camera.camera_y_sensor_size) ),
                img_size=(float(camera.camera_resolution_x) ,float(camera.camera_resolution_y)),
                focal=float(camera.camera_focal), ovlp=front_overlap,
                fixed_pitch=fixed_pitch,
                ref_alti=takeoff_altitude
                )
            else :
                prof1 = DroneOri(
                name=project_name+'_'+str(i), dsm=dsm, tfw=tfw,
                a_east=a_east, a_north=a_north, b_east=b_east, b_north=b_north,
                h=ground_distance, sensor_size=(float(camera.camera_x_sensor_size) ,float(camera.camera_y_sensor_size) ),
                img_size=(float(camera.camera_resolution_x) ,float(camera.camera_resolution_y)),
                focal=float(camera.camera_focal),
                ovlp=front_overlap,
                fixed_pitch=fixed_pitch,
                takeoff_pt=coordonates_transformer.transform(takeoff_point.lat, takeoff_point.lon)
                )
            prof1.dsm_profile()
            prof1.drone_orientations()
            ### create SVG with profile and drone orientations
            if DEBUG: prof1.draw_orientations(disp_linereg=True, disp_footp=True, disp_fov=True)
            ### create SVG map with profile 
            #if DEBUG: prof1.draw_map(shaded_dsm=shaded_dsm)

            # Add waypoints to main dict
            final_waypoint_dict+=prof1.export_ori()
            
    if takeoff_altitude :
        print(bg(F"\U0001f449 L'altitude de décollage utilisée est {prof1.ref_alti:.0f} m. A vérifier avec l'altitude GPS du drone \U0001f448",9))
    else:
        print(bg(F"\U0001f449 L'altitude calculée au point de décollage à partir du MNT est {prof1.ref_alti:.0f} m. A vérifier avec l'altitude GPS du drone \U0001f448",9))
            
    ################### kml from profils ##########################
    #from IPython import embed; embed()
    wp_extras=dict2djikml(final_waypoint_dict,project_name+'.kml',reverse_coordonates_transformer,altitude=prof1.ref_alti,onfinish=onfinish,speed = drone_speed)
    # tmp_wp=wp_extras[0]
    # print('######################@')
    # print('distances entre chaque WP consécutifs')
    # for wp in wp_extras:
    #     print(F'{tmp_wp.distanceTo(wp)}')
    #     tmp_wp=wp
    # print('######################@')
    # ################### html map ##########################
    for wp in wp_extras:
        Path_generator.extra_point.append(wp)

    # export map
    the_map = WaypointMap(takeoff_point)
    # on place les limites de la zone
    the_map.add_polygon(points=points, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

    # On ajoute les waypoint qui ont été trouvés a la carte
    for wp in Path_generator.waypoint_list:
        the_map.add_waypoint(wp, direction=False,
                             footprint=False, footprint_markers=False)

    the_map.add_colored_waypoint_path(Path_generator.waypoint_list)

    # Add extra points (for DEBUG)
    if DEBUG: print("#### Extra Points ####")
    for extra in Path_generator.extra_point:
        if DEBUG: print('extra text '+extra.text + '\t\tExtra point\t' + str(extra.lat)+ '\t'+str(extra.lon))
        the_map.add_extra(extra, text=extra.text)

    # Export html map
    the_map.export_to_file(project_name)

if __name__ == '__main__':
    main()
