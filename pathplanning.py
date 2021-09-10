#!/usr/bin/python
# -*- coding:utf-8 -*-
from math import *
import sys
# from branca.utilities import none_min
import geopy
import copy
from geopy.units import meters
from pygeodesy.units import Lat
import pyproj
from geopy import Point, distance
from geopy.distance import geodesic
from geopy import Point, distance

import waypoint
from utils import *
from waypoint import WayPoint
from waypointsmap import WaypointMap
from collections import deque

# https://nbviewer.jupyter.org/github/python-visualization/folium/blob/master/examples/Rotate_icon.ipynb
# rotation des icones

debug =False


class PathPlanning:
    "PathPlanning class"

    def __init__(self, points,  emprise_laterale, emprise_longitudinale, recouvrement_lat=0., recouvrement_lon=0.,orientation=None,start_point=None):
        """Pathplanning generates waypoints """

        self.points = points
        self.waypoint_list = []
        self.distances = []
        self.orientation = orientation
        self.start_point=start_point
        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres
        self.recouvrement_lat = recouvrement_lat  # pourcentage
        self.recouvrement_lon = recouvrement_lon  # pourcentage
        self.increment_lon = self.emprise_longitudinale*(1-recouvrement_lon)
        self.increment_lat = self.emprise_laterale*(1-recouvrement_lat)
 #       self.dist_long = geopy.distance.distance(meters=self.increment_lon)
#        self.dist_lat = geopy.distance.distance(meters=self.increment_lat)
    #    self.compute_distances()

    def compute_distances(self):
        """ compute distances between all points"""
        # last point == first point so don't compute
        for i in range(len(self.points)-1):
            self.distances.append(getDistance(
                self.points[i], self.points[i+1]))

    def generate_path(self, style):
        """choix du syle du path"""
        if style == "snail":
            self.generate_path_snail()
        elif style == "normal":
            self.generate_path_normal()

    def generate_path_snail_0(self):
        """ Crée un parcours de type escargot """
        # https://www.math10.com/en/geometry/geogebra/geogebra.html
        # point de départ
        tmp_point = [self.points[0][0], self.points[0][1]]
        finish = False
        j = 0
#        for j in range(0, 1):
        for j in range(0, 2):
          #      while (not finish):
            print("j "+str(j))
            # On parcours tous les points. Le dernier est en fait une copie du 1er pour plus de simplicité.
            # Il faut s'arreter à l'avant dernier point
            for i in range(len(self.points)-1):
                direction = getBearing(self.points[i], self.points[i+1])
                # print('i={} / {}'.format(i,len(self.points)))

                distance_a_couvrir = self.distances[i]

                # dernier point premier tour ok
                if (i == (len(self.points)-2)) and (j == 0):
                    angle_g = getAngle(
                        self.points[-2], self.points[0], self.points[1])
                    angle_d = getAngle(
                        self.points[0], self.points[1], self.points[2])
                    G = -(self.emprise_laterale/2)/(sin(radians(angle_g)))
                    distance_a_couvrir += G
                    print((self.emprise_laterale/2)/(sin(radians(angle_g))))
                # dernier point deuxieme tour
                elif (i == (len(self.points)-2)) and (j == 1):
                    angle_g = getAngle(
                        self.points[-2], self.points[0], self.points[1])
                    angle_d = getAngle(
                        self.points[0], self.points[1], self.points[2])
                    G = (self.emprise_laterale/2)*(cos(radians(angle_g)))
                    D = (self.emprise_laterale/2)/sin(radians(180-angle_d))
                    print('i {} angle_g= {} angle_d {} G {} D {}'.format(
                        i, angle_g, angle_d, G, D))
                    distance_a_couvrir -= G
                    distance_a_couvrir -= D

                # premier point deuxieme tour ok
                elif (j > 0) and (i == 0):
                    angle_g = getAngle(
                        self.points[-2], self.points[0], self.points[1])
                    angle_d = getAngle(
                        self.points[0], self.points[1], self.points[2])
                    G = (self.emprise_laterale/2)*(cos(radians(angle_g)))
                    D = (self.emprise_laterale/2)/sin(radians(180-angle_d))
                    print('i {} angle_g= {} angle_d {} G {} D {}'.format(
                        i, angle_g, angle_d, G, D))
                    distance_a_couvrir -= G
                    distance_a_couvrir -= D

                elif (j > 0) and (i > 0):
                    angle_g = getAngle(
                        self.points[i-1], self.points[i], self.points[i+1])
                    angle_d = getAngle(
                        self.points[i], self.points[i+1], self.points[i+2])
                    G = (self.emprise_laterale/2)*(cos(radians(angle_g)))
                    D = (self.emprise_laterale/2)/sin(radians(180-angle_d))
                    print('i {} angle_g= {} angle_d {} G {} D {}'.format(
                        i, angle_g, angle_d, G, D))
                    if angle_g < 90:
                        distance_a_couvrir -= G
                    else:
                        distance_a_couvrir += G
                    if angle_d < 90:
                        distance_a_couvrir -= D
                    else:
                        distance_a_couvrir += G

                if distance_a_couvrir < 0:
                    finish = True

                print('distance_a_couvrir {}m '.format(distance_a_couvrir))

                # self.waypoint_list.append(WayPoint([tmp_point[0], tmp_point[1]], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

                tmp_point = point_distance_bearing_to_new_point(
                    (tmp_point[0], tmp_point[1]), distance_a_couvrir, direction)

                self.waypoint_list.append(WayPoint(
                    [tmp_point[0], tmp_point[1]], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

            # j +=1
                # while distance_parcourue <= distance_a_couvrir:

                # tmp = dist.destination(point=Point(tmp_point), bearing=direction)
                #  tmp=point_distance_bearing_to_new_point((tmp_point[0],tmp_point[1]),self.increment_lon,direction)

                #  self.waypoint_list.append(
                #      WayPoint([tmp[0], tmp[1]], direction, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

                #  tmp_point = [tmp[0], tmp[1]]
                #  distance_parcourue += self.increment_lon

                # print('distance_a_couvrir {}m distance_parcourue {}m'.format(
                #   distance_a_couvrir, distance_parcourue))

    def generate_path_normal(self):
        """ path type allez-retour
        avec des parrallelles"""
        # point de départ
        indice_gauche = len(self.points)
        indice_droit = 1
        right = True
        limite_AB = .5
        finish = False
        not_in_right = False
        not_in_left = False

        tmp_A = self.points[0]
        tmp_B = self.points[1]
        # la direction initiale est celle de points[0] vers points[1]
        direction_right = getBearing(tmp_A, tmp_B)
        direction_left = getBearing(tmp_B, tmp_A)
                
        
        tmp = None
        # premiers point
        self.waypoint_list.append(WayPoint(
            tmp_A, direction_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

        self.waypoint_list.append(WayPoint(tmp_B, getBearing(
            tmp_B, self.points[indice_droit+1]),
            emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
        i = 0

        # on s'arrete quand cette distance est très petite
        while (getDistance(tmp_A, tmp_B) > limite_AB) and not finish:
            #print('i {}'.format(i))
            #if i == 15:
            #    break
            direction_ext_right = getBearing(
                tmp_B, self.points[indice_droit+1])
            direction_ext_left = getBearing(
                tmp_A, self.points[indice_gauche-1])

            #milieu_tmpA_tmpB = middlepoint(tmp_A, tmp_B)
            # Le point C est  sur la parralle à [tmp_A tmp_B]
            C = point_distance_bearing_to_new_point(
                tmp_A, self.increment_lat, direction_right+90)
            # le point D est sur la ligne C avec son bearing et à distance d=10m sans importance
            D = point_distance_bearing_to_new_point(
                C, 10, direction_left)
            
            # on trouve les points d'intersection de C avec son bearing et les segments latéraux
            # self.waypoint_list.append(WayPoint(
            #        C, direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale,text="C"))


#            intersect_right = intersect_points_bearings(
#                C, direction_right, tmp_B, direction_ext_right)
 
            intersect_right = intersect_four_points_(
                C, D, tmp_B, self.points[indice_droit+1])
 

            # verification que les points son bien sur les segments
            if not iswithin(intersect_right, tmp_B, self.points[indice_droit+1]):
                #not_in_right = True
                if debug : print('not in segment right')
                indice_droit += 1
                # Arret si le nouveau sommet est celui du coté opposé
                if (indice_droit == indice_gauche)or (indice_droit>len(self.points)-2):
                    if debug :  print('finish')
                    finish = True
                    break
                direction_ext_right = getBearing(
                    self.points[indice_droit], self.points[indice_droit+1])

                intersect_right = intersect_four_points_(
                C, D, self.points[indice_droit], self.points[indice_droit+1])
    
                if not iswithin(intersect_right, self.points[indice_droit], self.points[indice_droit+1]):
                    if debug :  print('Really not in segment right !')
                    not_in_right = True

            intersect_left = intersect_four_points_(
                C, D, tmp_A, self.points[indice_gauche-1])
            # si il n'y a pas d'intersection dans ce segment, on prend le suivant
            if not iswithin(intersect_left, tmp_A, self.points[indice_gauche-1]):
                if debug :  print('not in segment left')
                indice_gauche -= 1
                # Arret si le nouveau sommet est celui du coté opposé
                if (indice_droit == indice_gauche):
                    if debug :  print('finish')
                    finish = True
                    break
                direction_ext_left = getBearing(
                    self.points[indice_gauche], self.points[indice_gauche-1])

                intersect_left = intersect_four_points_(
                C, D, self.points[indice_gauche], self.points[indice_gauche-1])

                if not iswithin(intersect_left, self.points[indice_gauche], self.points[indice_gauche-1]):
                    if debug :  print('Really not in segment left !')
                    not_in_left = True

            if not_in_left and not_in_right:
                if debug : print('Finish !!!! ')
                finish = True
                #self.waypoint_list.append(WayPoint(
                #    C, direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale,text="C"))
                #self.waypoint_list.append(WayPoint(D                    , direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale,text="D"))

                break
            # arret en cas de pb
            if(intersect_right[0] < 0) or intersect_left[0] < 0:
                print('oups ! ')
                if right:
                    print('to_right')
                else:
                    print('to_left')
                self.waypoint_list.append(WayPoint(
                    C, direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

                print('C {}'.format(C))
                print('direction_right {}'.format(direction_right))
                print('indice_droit {} indice_gauche {}'.format(
                    indice_droit, indice_gauche))
                print('intersect_right {} intersect_left {}'.format(
                    intersect_right, intersect_left))
                print('tmp_A {} tmp_B {}'.format(tmp_A, tmp_B))

                break
            # on ajoute les points dans le bon ordre
            if right:
                self.waypoint_list.append(WayPoint(
                    intersect_right, direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
                self.waypoint_list.append(WayPoint(
                    intersect_left, direction_ext_left, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
            else:
                self.waypoint_list.append(WayPoint(
                    intersect_left, direction_ext_left, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
                self.waypoint_list.append(WayPoint(
                    intersect_right, direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

#            print('intersect_right {} intersect_left {}'.format(                intersect_right, intersect_left))
            tmp_A = intersect_left
            tmp_B = intersect_right
            right = not right
            i += 1
            not_in_left = False
            not_in_right = False

    def compute_length_and_turns(self):
        """ Compute the path length and turn numbers"""

        total_distance=0.0
        nb_turns= 0

        #start point
        distance_list = [self.start_point]

        for waypoint in self.waypoint_list:
            distance_list.append(waypoint.location)
        distance_list.append(self.start_point)
        #print(distance_list)
        
        # le "-1" pour ne pas aller trop loin
        for i in range(len(distance_list[:-1])):
            total_distance += getDistance(distance_list[i],distance_list[i+1])
            nb_turns +=1
        print('Total distance is {}m with {} turns'.format(total_distance,nb_turns))


    def export_to_kml(self):
        """ Export waypoints to kml for DJI UAV"""
        pass


def main(args):
    """la fonction main"""
    start_point_list=[]
    start_point_list.append((48.846383084057315, 2.356424447320463))
    start_point_list.append((48.84415899803569, 2.353495475053588))
    start_point_list.append((48.844599153918324, 2.355340339361402))
    start_point_list.append((48.84508549389749, 2.356311190101862))
    #start_point = (48.84361006276646, 2.3559057454019223)

    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84415592294359, 2.3565687535257593)
    E = (48.84395753653702, 2.355015706155173)
    F = (48.844565798460536, 2.3552507666007094)

 #   points = [E, A, B, C, D]
    points = deque([A, B, C, D, E])
    points = deque([ E,A,B, C, D])
#    points = deque([A, B, C, D, E,F])
#    points = [A, B, C, D]
    #points = deque( [A, B, C])

  
   # orientation = angle_EAB
    emprise_laterale = 50
    emprise_longitudinale = 20

    # on fait varier les start_point
    #    for start_point in start_point_list:

    start_point = start_point_list[1]
    #on fait varier le point de départ
    for i in range(len(points)):

        Path_generator = PathPlanning(points,  emprise_laterale,
                                    emprise_longitudinale, start_point=start_point ,recouvrement_lat=0.8, recouvrement_lon=0.8)
        # pp.find_best_orientation()
        # pp.GeneratePath("snail")
    #    Path_generator.generate_path_snail_0()
        Path_generator.generate_path_normal()
        the_map = WaypointMap(start_point)

        # on place les limites de la zone
        the_map.add_polygon(locations=points, color='#ff7800', fill=True,
                            fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

        # On ajoute les waypoint qui ont été trouvés a la carte
        for wp in Path_generator.waypoint_list:
            the_map.add_waypoint(wp, direction=False, footprint=False)

        the_map.add_path_waypoints(Path_generator.waypoint_list)

        # Exportation de la carte
        the_map.export_to_file()

        print('Start point is {}'.format(points[0]))
        # Path_generator.export_to_kml()
        Path_generator.compute_length_and_turns()

        # on fait tourner le 1er point
        points.rotate(1)
if __name__ == '__main__':
    main(sys.argv)