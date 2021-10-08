#!/usr/bin/python
# -*- coding:utf-8 -*-
from math import atan, degrees, sqrt
import folium
from gpxplotter import create_folium_map
from geopy import Point, distance
import geopy
import sys
from pygeodesy.dms import lonDMS

from pygeodesy.units import Lat
from utils import *
from pygeodesy.sphericalTrigonometry import LatLon 


class WayPoint:
    "Waypoint class"

    def __init__(self, point, orientation, lateral_footprint=100, longitudinal_footprint=50,wp_text=''):
        
        # point is a latlontri
        self.point = point
        self.latitude = self.point.lat
        self.longitude = self.point.lon
        self.location = [self.point.lat,self.point.lon]

        self.altitude_absolue_sol = None
        self.altitude_relative_drone = None
        self.text=str(wp_text)
        if len(self.text)>0:
            self.text=self.text+' '
        # self.hauteur_sol
        self.lateral_footprint = lateral_footprint  # en mètres
        self.longitudinal_footprint = longitudinal_footprint  # en mètres
        self.orientation = orientation
        if self.lateral_footprint != 0 and self.longitudinal_footprint != 0 :
            self.emprise_coordinates()
        
    def __str__(self):
        return ('[{} {}]'.format(self.latitude, self.longitude))

    def latlon(self):
        return (self.latitude, self.longitude)

    def footprint_intersection(self,waypoint_2,isclockwise):
        """
        Return intersection point of two waypoint footprints
        left intersection is isclockwise
        right intersection if is not isclockwise
        """
        
      #  print('WP1 {}  WP2  {}'.format(self.__str__(),waypoint_2.__str__()))
#        print('WP1 {} {} WP2 {} {}'.format(self.lat,self.lon,waypoint_2.lat,waypoint_2.lon))
        if isclockwise:
           # print( ' self.X0 {} self.X1 {} waypoint_2.X0 {} waypoint_2.X1 {}'.format(self.X0,self.X1,waypoint_2.X0,waypoint_2.X1))
            return doIntersect(self.X0_latlon,self.X1_latlon,waypoint_2.X0_latlon,waypoint_2.X1_latlon)
        else:
           # print( ' self.X2 {} self.X3 {} waypoint_2.X2 {} waypoint_2.X3 {}'.format(self.X2,self.X3,waypoint_2.X2,waypoint_2.X3))
            return doIntersect(self.X2_latlon,self.X3_latlon,waypoint_2.X2_latlon,waypoint_2.X3_latlon)
        

    def emprise_coordinates(self):
        """Détermine les coordonnées de l'emprise au sol à partir du point central, de l'orientation et des emprises
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
    
        self.X0_latlon = self.point.destination(sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),angle + self.orientation)
        self.X0 = [self.X0_latlon.lat,self.X0_latlon.lon]

        self.X1_latlon =  self.point.destination(sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),180-angle + self.orientation)
        self.X1 = [self.X1_latlon.lat,self.X1_latlon.lon]

        self.X2_latlon = self.point.destination( sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),180+angle + self.orientation)
        self.X2 = [self.X2_latlon.lat,self.X2_latlon.lon]

        self.X3_latlon=self.point.destination(sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),360-angle + self.orientation)
        self.X3 = [self.X3_latlon.lat,self.X3_latlon.lon]

def main(args):

    lat = 48.84482270388685
    lon = 2.3562098704389163

    IPGP = WayPoint(LatLon(lat, lon),90)
    print('X0 {} X1 {} X2 {} X3 {}'.format(IPGP.X0,IPGP.X1,IPGP.X2,IPGP.X3))


if __name__ == '__main__':
    main(sys.argv)
