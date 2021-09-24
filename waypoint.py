#!/usr/bin/python
# -*- coding:utf-8 -*-
from math import atan, degrees, sqrt
import folium
from gpxplotter import create_folium_map
from geopy import Point, distance
import geopy
import sys
from utils import *
from pygeodesy.sphericalTrigonometry import LatLon 


class WayPoint:
    "Waypoint class"

    def __init__(self, point, orientation, emprise_laterale=100, emprise_longitudinale=50,wp_text=''):
        
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
        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres
        self.orientation = orientation
        if self.emprise_laterale != 0 and self.emprise_longitudinale != 0 :
            self.emprise_coordinates()
        
    def __str__(self):
        return ('[{} {}]'.format(self.latitude, self.longitude))

    def latlon(self):
        return (self.latitude, self.longitude)

 
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
        
        self.delta_lat = self.emprise_laterale / 2
        self.delta_long = self.emprise_longitudinale / 2

        angle = degrees(atan(self.delta_lat/self.delta_long))

        #dist = geopy.distance.distance(kilometers=(
        #    sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long)/1000))

        #tmp = dist.destination(point=Point(
        #    self.latitude, self.longitude), bearing=angle + self.orientation)
#
        #print("angle "+str(angle + self.orientation))
        #self.X0 = [tmp.latitude, tmp.longitude]
        
        self.X0_latlon = point_distance_bearing_to_new_point_latlon(point= self.point,
        distance= sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),bearing=angle + self.orientation)

        self.X0 = [self.X0_latlon.lat,self.X0_latlon.lon]

        #tmp = dist.destination(point=Point(
        #    self.latitude, self.longitude), bearing=180-angle + self.orientation)

        #print("angle "+str(180-angle + self.orientation))
        self.X1_latlon = point_distance_bearing_to_new_point_latlon(point= self.point,
        distance= sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),bearing=180-angle + self.orientation)
        self.X1 = [self.X1_latlon.lat,self.X1_latlon.lon]

     #   tmp = dist.destination(point=Point(
     #       self.latitude, self.longitude), bearing=180+angle + self.orientation)
        #print("angle "+str(180+angle + self.orientation))
    #    self.X2 = [tmp.latitude, tmp.longitude]
        self.X2_latlon = point_distance_bearing_to_new_point_latlon(point= self.point,
        distance= sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),bearing=180+angle + self.orientation)
        self.X2 = [self.X2_latlon.lat,self.X2_latlon.lon]

  #      tmp = dist.destination(point=Point(
 #           self.latitude, self.longitude), bearing=360-angle + self.orientation)
        #print("angle "+str(360-angle + self.orientation))
#        self.X3 = [tmp.latitude, tmp.longitude]

        self.X3_latlon=point_distance_bearing_to_new_point_latlon(point= self.point,
        distance= sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long),bearing=360-angle + self.orientation)
        self.X3 = [self.X3_latlon.lat,self.X3_latlon.lon]

def main(args):

    lat = 48.84482270388685
    lon = 2.3562098704389163

    IPGP = WayPoint(LatLon(lat, lon),90)
    print(IPGP.X0)
    print(IPGP.X1)
    print(IPGP.X2)
    print(IPGP.X3)


if __name__ == '__main__':
    main(sys.argv)

# text = 'text'
# edges = [upper_left, upper_right, lower_right, lower_left]
# map_osm = folium.Map(location=[latty, longy], zoom_start=14)
# map_osm.add_child(folium.vector_layers.Polygon(locations=edges, color=line_color, fill_color=fill_color,
#                                                weight=weight, popup=(folium.Popup(text))))
