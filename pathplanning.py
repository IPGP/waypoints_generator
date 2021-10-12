#!/usr/bin/python
# -*- coding:utf-8 -*-
from math import tan, radians
import sys
from collections import deque
from itertools import cycle
import time
from numpy import Inf
from pygeodesy.sphericalTrigonometry import LatLon
from pygeodesy.sphericalNvector  import intersection, LatLon as LatLonS
from pygeodesy.points import isclockwise, isconvex, centroidOf
from utils import getAnglelatlon
from waypoint import WayPoint
from waypointsmap import WaypointMap

# https://nbviewer.jupyter.org/github/python-visualization/folium/blob/master/examples/Rotate_icon.ipynb
# rotation des icones
# Faire des schémas
# https://www.math10.com/en/geometry/geogebra/fullscreen.html


DEBUG = False
#DEBUG = True


class PathPlanning:
    "PathPlanning class"

    def __init__(self, points,  bearing=0, lateral_footprint=0, longitudinal_footprint=0,
     percent_recouvrement_lat=0., percent_recouvrement_lon=0., orientation=None, start_point=None):
        """Pathplanning generates waypoints """

        if not isconvex(points):
            print("points must form a convex shape")
            sys.exit(-1)
        self.start_time = time.time_ns()


        self.points = points
        self.CENTROID_LATLON_S = centroidOf(self.points, LatLon=LatLonS)
        self.CENTROID_LATLON = centroidOf(self.points, LatLon=LatLon)

        self.nb_points = len(self.points)
        self.bearing = bearing
#        if DEBUG: print('Les points de ce pathplanning sont {}'.format(self.points))
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

        # Create all the segments of the area
        self.area_segments = []
        i = 0
        point_cycle = cycle(self.points)
        next_point = next(point_cycle)

        ## All the points in LatLonS for reference
        self.points_LatLonS = []

        while i < self.nb_points:
            this_point, next_point = next_point, next(point_cycle)
            self.area_segments.append([LatLonS(this_point.lat, this_point.lon), LatLonS(next_point.lat, next_point.lon)])
            self.points_LatLonS.append(LatLonS(this_point.lat, this_point.lon))
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

        # Find the best starting point for self.bearing

        ref_point = self.points[0]
        distance_max_to_centroid = 0
        projection_point = None

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

        # find direction from ref_point to centroid
        direction_to_centroid = projection_point.compassAngleTo(self.CENTROID_LATLON)

        # If we start from self.farest_summit
        intersection_list = []
        intersection_exist = True

        i = 1

        # We search until we do not have intersections points
        while intersection_exist :
            new_point = ref_point.destination(i*self.increment_lat, direction_to_centroid)
            new_point_LatLonS = LatLonS(new_point.lat, new_point.lon)
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
                elif intersection_point_B and intersection_point_B.iswithin(segment[0], segment[1]):
                    intersection_list.append(intersection_point_B)
                    intersection_exist = True
                    nb_intersection +=1
                # If 2 intersections have been found, we break this loop
                if nb_intersection == 2:
                    break

            i += 1
        #print(F"Paf\t{time.time_ns()-self.start_time}")

        # we need to reorder the pairs of point
        flip_left_right = False

        # the summit will be the first WP
        self.waypoint_list.append(WayPoint(
            ref_point, self.bearing, lateral_footprint=self.lateral_footprint, longitudinal_footprint=self.longitudinal_footprint))

        self.paired_waypoints_list.append([[ref_point.lat, ref_point.lon], []])

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
    """
    Martinique
    """
    start_point = LatLon(14.810431373395028, -61.176753253004485)
    start_point.text = 'Start'
    a = LatLon(14.801717517497558, -61.17830598375636)
    b = LatLon(14.803184637915658, -61.177470922032136)
    c = LatLon(14.804110982101069, -61.176259107347896)
    d = LatLon(14.80476281073443, -61.175343978459765)
    e = LatLon(14.804147551878703, -61.17414211429372)
    f = LatLon(14.802389075700889, -61.175630772903205)
    g = LatLon(14.801758424759862, -61.176496729696545)

#    points = deque([f, e, d, c, b, a])
#    points = deque([a,f, e, d, c, b ])
#    points = deque([a, b, c])
#    points = deque([a, b, c, d])
    points = deque([a, b, c, d, e, f, g])

    lateral_footprint = 80
    longitudinal_footprint = 50

    Path_generator = PathPlanning(points=points,  bearing=140, lateral_footprint=lateral_footprint,
                                  longitudinal_footprint=longitudinal_footprint, start_point=start_point, percent_recouvrement_lat=0.6, percent_recouvrement_lon=0.80)

    Path_generator.extra_point.append(start_point)
    Path_generator.generate_path_normal_plus()

    # export map
    the_map = WaypointMap(start_point)
    # on place les limites de la zone
    the_map.add_polygon(points=points, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

    # On ajoute les waypoint qui ont été trouvés a la carte
    for wp in Path_generator.waypoint_list:
        the_map.add_waypoint(wp, direction=False,
                             footprint=False, footprint_markers=False)

    the_map.add_colored_waypoint_path(Path_generator.waypoint_list)

    # Add extra points (for DEBUG)
    for extra in Path_generator.extra_point:
        the_map.add_extra(extra, text=extra.text)
        #print('extra text '+extra.text + '\t\tExtra point\t' + str(extra.lat)+ '\t'+str(extra.lon))

    # Exportation de la carte
    the_map.export_to_file('normal_plus')
    # Path_generator.export_to_kml()

    wp_list = []
    for wp in Path_generator.waypoint_list:
        wp_list.append(wp.latlon())

   # from IPython import embed; embed()


if __name__ == '__main__':
    main()
