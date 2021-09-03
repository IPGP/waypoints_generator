#!/usr/bin/python
# -*- coding:utf-8 -*-
import sys
from branca.utilities import none_min
import geopy
from geopy.units import meters
import pyproj
from geopy import Point, distance
from geopy.distance import geodesic
from geopy import Point, distance

import waypoint
from utils import *
from waypoint import WayPoint
from waypointsmap import WaypointMap

# https://nbviewer.jupyter.org/github/python-visualization/folium/blob/master/examples/Rotate_icon.ipynb
# rotation des icones


class Corner:
    def __init__(self, coordonnes):
        self.coordonnes = coordonnes
        self.distance_to_next_corner
    pass

    @property
    def coordonnes(self):
        return self._coordonnes

    @coordonnes.setter
    def coordonnes(self, coordonnes):
        self._coordonnes = coordonnes


class PathPlanning:
    "PathPlanning class"

#    def __init__(self, points, orientation, emprise_laterale, emprise_longitudinale,recouvrement_lat=0.7, recouvrement_lon=0.7):
    def __init__(self, points, orientation, emprise_laterale, emprise_longitudinale,recouvrement_lat=0., recouvrement_lon=0.):
        """Pathplanning generates waypoints """

        # On rajoute le premier point a la fin pour plus de simplicité
        self.points = points
        self.points.append(points[0])

        self.waypoint_list = []

        self.orientation = orientation

        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres
        self.recouvrement_lat=recouvrement_lat # pourcentage
        self.recouvrement_lon=recouvrement_lon # pourcentage
        self.increment_lon = self.emprise_longitudinale*(1-recouvrement_lon)
        self.increment_lat = self.emprise_laterale*(1-recouvrement_lat)
        self.dist_long = geopy.distance.distance(meters=self.increment_lon)
        self.dist_lat = geopy.distance.distance(meters=self.increment_lat)


    def generate_path(self, style):
        """choix du syle du path"""
        if style == "snail":
            self.generate_path_snail()
        elif style == "normal":
            self.generate_path_normal()

    def generate_path_snail_0(self):
        """ Crée un parcours de type escargot """
        #https://www.math10.com/en/geometry/geogebra/geogebra.html
        # point de départ
        tmp_point = [self.points[0][0],self.points[0][1]]

        for j in range(0,1):
            print("j "+str(j))
            # On parcours tous les points. Le dernier est en fait une copie du 1er pour plus de simplicité.
            # Il faut s'arreter à l'avant dernier point
            for i in range(len(self.points)-1):
            
                # sur le dernier tour, on fait un coup de moins
                if i == (len(self.points)-2):
                    distance_a_couvrir = geodesic(self.points[i], self.points[i+1]).meters-(j+1)*self.increment_lat
                else:
                    distance_a_couvrir = geodesic(self.points[i], self.points[i+1]).meters
                
                #distance_a_couvrir = geodesic(self.points[i], self.points[i+1]).meters-j*self.increment_lon
    
                distance_parcourue = 0
                direction = getBearing(self.points[i], self.points[i+1])
                #print('direction {} distance_a_couvrir {}m distance_parcourue {}m'.format(direction,
                #                                                                        distance_a_couvrir, distance_parcourue))

                # self.waypoint_list.append(WayPoint(
                #    self.points[0], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

#                tmp_point = [self.points[i][0], self.points[i][0]]
                #tmp_point = point_distance_bearing_to_new_point((self.points[i][0],self.points[i][1]),self.increment_lat*j,direction+90)
                tmp_point = point_distance_bearing_to_new_point((tmp_point[0],tmp_point[1]),distance_a_couvrir,direction)
                

                self.waypoint_list.append(WayPoint([tmp_point[0], tmp_point[1]], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
                self.waypoint_list.append(WayPoint([tmp_point[0], tmp_point[1]], direction+90, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

                #while distance_parcourue <= distance_a_couvrir:

                    #tmp = dist.destination(point=Point(tmp_point), bearing=direction)
                  #  tmp=point_distance_bearing_to_new_point((tmp_point[0],tmp_point[1]),self.increment_lon,direction)

                  #  self.waypoint_list.append(
                  #      WayPoint([tmp[0], tmp[1]], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

                  #  tmp_point = [tmp[0], tmp[1]]
                  #  distance_parcourue += self.increment_lon

                   # print('distance_a_couvrir {}m distance_parcourue {}m'.format(
                     #   distance_a_couvrir, distance_parcourue))

    def generate_path_normal(self):
        """ path type allez-retour"""

    def export_to_kml(self):
        """ Export waypoints to kml for DJI UAV"""
        pass


def main(args):
    """la fonction main"""

    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84415592294359, 2.3565687535257593)
    E = (48.84395753653702, 2.355015706155173)

#    points = [E, A, B, C, D]
    points = [A, B, C, D]

    # Calcul des distances
    AB = geodesic(A, B).m
    BC = geodesic(B, C).m
    CD = geodesic(C, D).m
    DA = geodesic(D, A).m
    BD = geodesic(B, D).m
    AC = geodesic(A, C).m

    # Calcul des angles
    angle_ABC = getAngle(A, B, C)
    angle_BCD = getAngle(B, C, D)
    angle_CDE = getAngle(C, D, E)
    angle_DEA = getAngle(D, E, A)
    angle_EAB = getAngle(E, A, B)

    orientation = angle_EAB
    emprise_laterale = 80
#    emprise_laterale = 50
    emprise_longitudinale = 50

    Path_generator = PathPlanning(points, orientation, emprise_laterale,
                                  emprise_longitudinale,0.8,0.8)
    # pp.find_best_orientation()
    # pp.GeneratePath("snail")
    Path_generator.generate_path_snail_0()

    the_map = WaypointMap()

    the_map.add_polygon(locations=points, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

    # On ajoute les waypoint qui ont été trouvés a la carte
    for wp in Path_generator.waypoint_list:
        the_map.add_waypoint(wp)

    the_map.add_path_waypoints(Path_generator.waypoint_list)

    # Exportation de la carte
    the_map.export_to_file()

    Path_generator.export_to_kml()


if __name__ == '__main__':
    main(sys.argv)
