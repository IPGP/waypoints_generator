#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from math import atan, degrees, sqrt
from pygeodesy.sphericalTrigonometry import LatLon
from pygeodesy.sphericalNvector import LatLon as LatLonS
from utils import do_intersect


class WayPoint:
    "Waypoint class"

    def __init__(self, lat, lon, orientation=None,
                 lateral_footprint=0, longitudinal_footprint=0, wp_text='', alt=None):

        # point is a latlontri
        self.latlon = LatLon(lat, lon)
        self.latlons = LatLonS(lat, lon)
        self.latitude = lat
        self.longitude = lon
        self.lat = lat
        self.lon = lon
        self.location = [lat, lon]
        self.altitude_absolue_sol = None
        self.altitude_relative_drone = alt
        self.text = str(wp_text)
        if len(self.text) > 0:
            self.text = self.text+' '
        # self.hauteur_sol
        self.lateral_footprint = lateral_footprint  # en mètres
        self.longitudinal_footprint = longitudinal_footprint  # en mètres
        self.orientation = orientation
        if self.lateral_footprint != 0 and self.longitudinal_footprint != 0:
            self.emprise_coordinates()

    def __str__(self):
        return F"[{self.latitude} {self.longitude}]"

    def latlon(self):
        return (self.latitude, self.longitude)

    def footprint_intersection(self, waypoint_2, isclockwise):
        """
        Return intersection point of two waypoint footprints
        left intersection is isclockwise
        right intersection if is not isclockwise
        """

        if isclockwise:
            return do_intersect(self.x0_latlon, self.x1_latlon,
                                waypoint_2.X0_latlon, waypoint_2.X1_latlon)
        return do_intersect(self.x2_latlon, self.x3_latlon,
                                waypoint_2.X2_latlon, waypoint_2.X3_latlon)

    def emprise_coordinates(self):
        """Détermine les coordonnées de l'emprise au sol à partir
        du point central, de l'orientation et des emprises
                   5 | 5
        x3,y3 ---------------- x0,y0
            |                 |
         10 |        ^        |
          – |        |        |
         10 |        X        |
            |                 |
            |                 |
        x2,y2 ---------------- x1,y1

        """

        self.delta_lat = self.lateral_footprint / 2
        self.delta_long = self.longitudinal_footprint / 2

        angle = degrees(atan(self.delta_lat/self.delta_long))

        self.x0_latlon = self.latlons.destination(sqrt(
            self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),
            angle + self.orientation)
        self.x0 = [self.x0_latlon.lat, self.x0_latlon.lon]

        self.x1_latlon = self.latlons.destination(sqrt(
            self.delta_lat * self.delta_lat+self.delta_long * self.delta_long), 180-angle + self.orientation)
        self.x1 = [self.x1_latlon.lat, self.x1_latlon.lon]

        self.x2_latlon = self.latlons.destination(sqrt(
            self.delta_lat * self.delta_lat+self.delta_long * self.delta_long), 180+angle + self.orientation)
        self.x2 = [self.x2_latlon.lat, self.x2_latlon.lon]

        self.x3_latlon = self.latlons.destination(sqrt(
            self.delta_lat * self.delta_lat+self.delta_long * self.delta_long), 360-angle + self.orientation)
        self.x3 = [self.x3_latlon.lat, self.x3_latlon.lon]
