from math import atan, degrees, sqrt
import folium
from gpxplotter import create_folium_map
from geopy import Point, distance
import geopy
import sys


class WayPoint:
    "Waypoint class"

    def __init__(self, location, orientation, emprise_laterale=100, emprise_longitudinale=50):
        self.location = location
        self.latitude = self.location[0]
        self.longitude = self.location[1]
        self.altitude_absolue_sol = None
        self.altitude_relative_drone = None
        # self.hauteur_sol
        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres
        self.orientation = orientation
        self.emprise_coordinates()

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

        dist = geopy.distance.distance(kilometers=(
            sqrt(self.delta_lat * self.delta_lat+self.delta_long * self.delta_long)/1000))

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=angle + self.orientation)
#
        #print("angle "+str(angle + self.orientation))
        self.X0 = [tmp.latitude, tmp.longitude]

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=180-angle + self.orientation)

        #print("angle "+str(180-angle + self.orientation))
        self.X1 = [tmp.latitude, tmp.longitude]

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=180+angle + self.orientation)
        #print("angle "+str(180+angle + self.orientation))
        self.X2 = [tmp.latitude, tmp.longitude]

        tmp = dist.destination(point=Point(
            self.latitude, self.longitude), bearing=360-angle + self.orientation)
        #print("angle "+str(360-angle + self.orientation))
        self.X3 = [tmp.latitude, tmp.longitude]


def main(args):

    lat = 48.84482270388685
    lon = 2.3562098704389163

    IPGP = WayPoint([lat, lon])
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
