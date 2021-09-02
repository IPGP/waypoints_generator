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
from utils import getAngle, getBearing
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

    def __init__(self, points, orientation, emprise_laterale, emprise_longitudinale):
        """Pathplanning generates waypoints """

        # On rajoute le premier point a la fin pour plus de simplicité
        self.points = points
        self.points.append(points[0])

        self.waypoint_list = []

        self.orientation = orientation

        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres

    def generate_path(self, style):
        """choix du syle du path"""
        if style == "snail":
            self.generate_path_snail()
        elif style == "normal":
            self.generate_path_normal()

    def generate_path_snail_0(self):
        """ Crée un parcours de type escargot """

        # On parcours tous les points. Le dernier est en fait une copie du 1er pour plus de simplicité.
        # Il faut s'arreter à l'avant dernier point
        for i in range(len(self.points)-1):
            distance_a_couvrir = geodesic(
                self.points[i], self.points[i+1]).meters
            distance_parcourue = 0
            increment = self.emprise_longitudinale*0.5
            direction = getBearing(self.points[i], self.points[i+1])
            print('direction {} distance_a_couvrir {}m distance_parcourue {}m'.format(direction,
                                                                                      distance_a_couvrir, distance_parcourue))

            # self.waypoint_list.append(WayPoint(
            #    self.points[0], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

            dist = geopy.distance.distance(meters=increment)
            tmp_point = [self.points[i][0], self.points[i][1]]

            while distance_parcourue <= distance_a_couvrir:

                tmp = dist.destination(point=Point(
                    tmp_point), bearing=direction)
                self.waypoint_list.append(
                    WayPoint([tmp.latitude, tmp.longitude], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

                tmp_point = [tmp.latitude, tmp.longitude]
                distance_parcourue += increment
                print('distance_a_couvrir {}m distance_parcourue {}m'.format(
                    distance_a_couvrir, distance_parcourue))

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

    points = [E, A, B, C, D]
#    points = [A, B, C, D]

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
    emprise_laterale = 30
    emprise_longitudinale = 15

    Path_generator = PathPlanning(points, orientation, emprise_laterale,
                                  emprise_longitudinale)
    # pp.find_best_orientation()
    # pp.GeneratePath("snail")
    Path_generator.generate_path_snail_0()

    the_map = WaypointMap()

    the_map.add_polygon(locations=points, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

    # On ajoute les waypoint qui ont été trouvés a la carte
    for wp in Path_generator.waypoint_list:
        the_map.add_waypoint(wp)

    # Exportation de la carte
    the_map.export_to_file()

    Path_generator.export_to_kml()


if __name__ == '__main__':
    main(sys.argv)
