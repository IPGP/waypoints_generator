from math import atan, degrees, sqrt
from geopy import Point, distance
import geopy
from waypointsmap import WaypointMap
import sys


class Waypoint:
    "Waypoint class"

    def __init__(self, latitude, longitude, altitude_absolue_sol=None, altitude_relative_drone=None,
                 hauteur_sol=None, emprise_laterale=100, emprise_longitudinale=50, direction=0, emprise=None, name=""):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude_absolue_sol = altitude_absolue_sol
        self.altitude_relative_drone = altitude_relative_drone
        self.hauteur_sol = hauteur_sol
        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres
        self.direction = direction
        self.emprise = emprise
        self.name = name

    def compute_footprint(self):
        """Détermine les coordonnées de l'emprise au sol à partir du point central
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
        delta_lat = self.emprise_laterale / 2
        delta_long = self.emprise_longitudinale / 2

        angle = degrees(atan(delta_lat/delta_long))

        dist = geopy.distance.distance(kilometers=(
            sqrt(delta_lat * delta_lat+delta_long * delta_long)/1000))

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=angle + self.direction)
        #        print("angle "+str(angle + self.direction))
        x0, y0, *_ = tmp

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=180-angle + self.direction)
        # print("angle "+str(180-angle + self.direction))
        x1, y1, *_ = tmp

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=180+angle + self.direction)
        # print("angle "+str(180+angle + self.direction))

        x2, y2, *_ = tmp
        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=360-angle + self.direction)

        x3, y3, *_ = tmp

        print(self.latitude, " ", self.longitude)
        print(x0, " ", y0)
        print(x1, " ", y1)
        print(x2, " ", y2)
        print(x3, " ", y3)

        self.emprise = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]


def main(args):

    LAT = 48.84482270388685
    LON = 2.3562098704389163

    X0 = 48.84643250706535
    Y0 = 2.3527444567404943
    X1 = 48.84350234422499
    Y1 = 2.358956452357875

    the_map = WaypointMap()
    the_map.add_area_of_interest([(X0, Y0),  (X0, Y1), (X1, Y1), (X1, Y0)])

    IPGP = waypoint(latitude=LAT,  longitude=LON, name="Center", direction=30)

    IPGP.compute_footprint()
    the_map.add_wapypoint(
        location=[LAT, LON], color='blue', popup_text="", icon='fa-map-pin')

    the_map.add_wapypoint(
        location=IPGP.emprise[0], color='red', popup_text="X0", icon='fa-map-pin')
    the_map.add_wapypoint(
        location=IPGP.emprise[1], color='green', popup_text="X1", icon='fa-map-pin')
    the_map.add_wapypoint(
        location=IPGP.emprise[2], color='green', popup_text="X2", icon='fa-map-pin')
    the_map.add_wapypoint(
        location=IPGP.emprise[3], color='red', popup_text="X3", icon='fa-map-pin')

    the_map.add_polygon(locations=IPGP.emprise, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup=IPGP.name)
    the_map.export_to_file()


if __name__ == '__main__':
    main(sys.argv)
