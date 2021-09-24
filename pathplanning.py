#!/usr/bin/python
# -*- coding:utf-8 -*-
from math import *
import sys
from IPython.terminal.embed import InteractiveShellEmbed
import geopy
import copy
from geopy.units import meters
from numpy import angle
from pygeodesy.units import Lat
from pygeodesy.sphericalTrigonometry import LatLon
from pygeodesy.sphericalNvector import LatLon as LatLonsphericalNvector
from geopy.distance import geodesic
from geopy import Point, distance
from pygeodesy.points import *
from pygeodesy.points import isclockwise, isconvex, centroidOf
from itertools import cycle

import waypoint
from utils import *
from waypoint import WayPoint
from waypointsmap import WaypointMap
from collections import deque
from copy import copy

# https://nbviewer.jupyter.org/github/python-visualization/folium/blob/master/examples/Rotate_icon.ipynb
# rotation des icones
# Faire des schémas
# https://www.math10.com/en/geometry/geogebra/fullscreen.html


debug = False
debug = True


class PathPlanning:
    "PathPlanning class"

    def __init__(self, points,  emprise_laterale, emprise_longitudinale, recouvrement_lat=0., recouvrement_lon=0., orientation=None, start_point=None):
        """Pathplanning generates waypoints """

#        if not isconvex(points):
#            print("points must form a convex shape")
#            sys.exit(-1)

        self.points = points
        self.nb_points = len(self.points)
        if debug: print('Les points de ce pathplanning sont {}'.format(self.points))
        self.waypoint_list = []
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
        self.emprise_laterale = emprise_laterale  # en mètres
        self.emprise_longitudinale = emprise_longitudinale  # en mètres
        self.recouvrement_lat = recouvrement_lat  # pourcentage
        self.recouvrement_lon = recouvrement_lon  # pourcentage
        self.increment_lon = self.emprise_longitudinale*(1-recouvrement_lon)
        self.increment_lat = self.emprise_laterale*(1-recouvrement_lat)
        #print('recouvrement_lat {} self.emprise_laterale {} self.increment_lat {}'.format(recouvrement_lat,self.emprise_laterale,self.increment_lat))
        self.compute_distances_and_bearings(self.points)

    def old_compute_distances_and_bearings(self,input_points):
        """ compute distances between all points"""
        # last point == first point so don't compute
        points = copy(input_points)
        points.append(input_points[0])

        for i in range(len(points)-1):
            if debug: print('point lat {} lon {}'.format(points[i].lat, points[i].lon))
            self.distances.append(points[i].distanceTo(points[i+1]))
            self.bearings.append(points[i].compassAngleTo(points[i+1]))
            if i==0:
                points[i].angle=getAnglelatlon(points[i-2],points[i],points[i+1])
                points[i].l=points[i].distanceTo(points[i+1])
            else:
                points[i].angle=getAnglelatlon(points[i-1],points[i],points[i+1])
                points[i].l=points[i].distanceTo(points[i+1])

        if debug:
            print('bearings {}'.format(self.bearings))
        if debug:
            print('distances {}'.format(self.distances))

        sys.exit()
 

    def compute_distances_and_bearings(self,input_points):
        """ compute distances between all points"""
        i=0
        point_cycle = cycle(self.points)
        next_point = next(point_cycle)
        preceding_point=self.points[-1]
        while i<self.nb_points:
            this_point, next_point = next_point, next(point_cycle)
            if debug: print('point lat {} lon {}'.format(this_point.lat, this_point.lon))
            self.distances.append(this_point.distanceTo(next_point))
            self.bearings.append(this_point.compassAngleTo(next_point))
            this_point.angle=getAnglelatlon(preceding_point,this_point,next_point)
            this_point.bearing=this_point.compassAngleTo(next_point)
            this_point.l=this_point.distanceTo(next_point)
            preceding_point = this_point
            i +=1
        if debug: print('bearings {}'.format(self.bearings))
        if debug: print('distances {}'.format(self.distances))



    def generate_path(self, style):
        """choix du syle du path"""
        self.style=style
        if style == "snail":
            self.generate_path_snail()
        elif style == "normal":
            self.generate_path_normal()
        elif style == "espiral":
            self.generate_path_espiral()
            



    def generate_path_espiral(self):
        """ Crée un parcours de type escargot """
        
        #   for point in self.points:
        #    print('Point {} {} Angle {} Distance au suivant {}'.format(point.lat ,point.lon,point.angle,point.l))
        
        self.centroid =centroidOf(self.points,LatLon=LatLon)
 
        #print('Centroid {} {} '.format(self.centroid.lat ,self.centroid.lon))
        #self.waypoint_list.append(WayPoint(self.centroid,0, emprise_laterale=0, emprise_longitudinale=0, wp_text="Centroid"))
                            
        # Find shortest distance between self.centroid and each segments
        distance_cp_min = float('inf')
        i=0
        point_cycle = cycle(self.points)
        next_point = next(point_cycle)

        while i<self.nb_points:
            this_point, next_point = next_point, next(point_cycle)
            tmp_distance_point =(self.centroid.nearestOn(this_point, next_point))
            tmp_distance=tmp_distance_point.distanceTo(self.centroid)
            #print('distance {} {} to {} {} is {}'.format(this_point.lat,this_point.lon,next_point.lat,next_point.lon,tmp_distance))
            if tmp_distance <distance_cp_min : distance_cp_min = tmp_distance
            i +=1
        print('Centroid to each segment minimum distance is {}'.format(distance_cp_min))

        ovx=self.emprise_laterale*self.recouvrement_lat
        #### NB of Rings
        dr = self.emprise_laterale-ovx
        nb_of_rings = ceil((distance_cp_min - ovx)/(dr))

        if nb_of_rings == 1:
            print("un seul ring ! => exit")
            sys.exit()

        print('nb_of_rings is {}'.format(nb_of_rings))

        #print('old ovx {} old dr {}'.format(self.increment_lat,dr))
        new_ovx = (nb_of_rings*self.emprise_laterale-distance_cp_min)/(nb_of_rings-1)
        new_dr = (distance_cp_min-self.emprise_laterale)/(nb_of_rings-1)
        
        #print('new ovx {} new dr {}'.format(ovx,dr))

        j = 0
        first_point = True
        new_point = None

        for ring in range(nb_of_rings):
            last_waypoints=[]
            first_waypoints=[]

            for point in self.points:
                
                if first_point :
                   # print('max is {}'.format(max([0,-self.emprise_laterale/tan(radians(point.angle))])))
                    d=point.l+max([0,-self.emprise_laterale/tan(radians(point.angle))])

                else:
                    d=point.l
   
                ovy=self.emprise_longitudinale*self.recouvrement_lon
                nb_waypoints_per_segment=ceil((d-ovy)/(self.emprise_longitudinale-ovy))
                new_ovy = (nb_waypoints_per_segment*self.emprise_longitudinale-point.l)/(nb_waypoints_per_segment-1)
                new_dw = ((d - self.emprise_longitudinale)/(nb_waypoints_per_segment-1))
                
                #print('Nb de waypoints {} new_dw is {}'.format(nb_waypoints_per_segment,new_dw))
                #print('Ly {} new_ovy{}'.format(self.emprise_longitudinale,new_ovy))

                # pourquoi rajouter +1 ? Sinon ca ne marche pas...
                for i in range(nb_waypoints_per_segment):
                    
                    if i == 0 :
                        if first_point:
                            first_point = False
                            new_point= point.destination((self.emprise_laterale/2), point.bearing+self.othogonal_increment)
                        else:    
                            new_point_tmp= point.destination((self.emprise_laterale/2), point.bearing+self.othogonal_increment)
                            new_point= new_point_tmp.destination((self.emprise_longitudinale/2), point.bearing)
                        first_waypoints.append(WayPoint(new_point,point.bearing, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
                    else:
                        new_point= last_point.destination(new_dw, point.bearing)

                    if i == nb_waypoints_per_segment-1 :
                        last_waypoints.append(WayPoint(new_point,point.bearing, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

                    self.waypoint_list.append(WayPoint(new_point,point.bearing, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
                    last_point=new_point
                    j+=1

            ### Calcul des nouveaux vertices :
            new_points=[]
            i=0
            last_cycle = cycle(last_waypoints)
            first_cycle = cycle(first_waypoints)
            first= next(first_cycle)

            while i<len(last_waypoints):
                last= next(last_cycle)
                first= next(first_cycle)

                inter = last.footprint_intersection(first,self.isclockwise)
                if inter:
                    new_points.append((inter))
                    #self.waypoint_list.append(WayPoint(inter,point.bearing, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale,wp_text='inter'))
                else:
                    print('probleme pas d intersection')
                i +=1
            if ring == 0 :

                return
                



    def generate_path_snail(self):
        """ Crée un parcours de type escargot """

        # le premier point est celui qui est donné dans la liste
        tmp_point = self.points[0]

        round_nb = 0
        j = 0
        # keep the created segments with their offset
        segments_list = []
        CD_segments_list = []

        finish=False

        # Each time we create a new waypoint,  lock_points is initialise whith False
        # the stop condition
        lock_points = [False  for i in range(self.nb_points)]

        # Si finish 
        # Any fait un OR sur toute la liste
        # OR operation - Using any()
        # AND Operation - Using all()
        finish_condition=all(lock_points)

        # on parcours tous les points
#        while not finish_condition:
        while not finish:
            # i va décrire tous les points
            i = 0
            for point in self.points:
                print(i)
                if i == self.nb_points-1:
                    next_point = self.points[0]
                else:
                    next_point = self.points[i+1]
                
                # On ne fait rien pour le premier point
                j += 1
                if round_nb == 0:
                    new_point = point
                    #print('new_point is {} points[i] is {}'.format(new_point,self.points[i]))

                    # on change pour le dernier point
                    if i == self.nb_points:
                        new_point = next_point.destination(50, self.bearings[i]+180)

                    #from IPython import embed; embed()

                    self.waypoint_list.append(WayPoint(new_point, self.bearings[i],
                                                       emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text=i))

                    C = point.destination(self.increment_lat*round_nb, self.bearings[i]+self.othogonal_increment)
                    D = C.destination( 50, self.bearings[i])
                    #print('distance entre C et tmp_point {}'.format(distance_to_line(tmp_point, C, D)))

                # After first rounf of all points
                else:
                    # middle_of_base_segment is on the middle of [point next_point] segment
                    middle_of_base_segment = point.destination( self.distances[i]/2, self.bearings[i])

                    # C is on the parrall line to [point next_point] shifted of self.increment_lat*round_nb
                    C = middle_of_base_segment.destination( self.increment_lat*round_nb, self.bearings[i]+self.othogonal_increment)

                    # D is on the "C line". 50 meters has no real meaning except for creating a second point on the line
                    D = C.destination( 50, self.bearings[i])

                    #print('distance entre C et tmp_point {}'.format(distance_to_line(tmp_point, C, D)))
                    # new point is a potential waypoint, and the intersection of [CD] and last created waypoint with bearing of current point 
                    new_point = intersect_points_bearings_latlon(tmp_point, self.bearings[i-1], C, self.bearings[i])


                    # E is  on the parrall line to [next_point new_next_point] shifted of self.increment_lat*round_nb
                    # F is on the same "E line"'
                    # new_point_bis is the intersection point if it exists of [EF] and last created waypoint with bearing of next point 
                    if i<self.nb_points-1:
                        E = next_point.destination( self.increment_lat*round_nb, self.bearings[i+1]+self.othogonal_increment)
                        # 50m est une valeur completement aléatoire !
                        F = E.destination( 50, self.bearings[i+1])
                        new_point_bis = intersect_points_bearings_latlon(tmp_point, self.bearings[i-1], E, self.bearings[i+1])

                    #last_point
                    else:
                        E = next_point.destination( self.increment_lat*(round_nb+1), self.bearings[0]+self.othogonal_increment)
                        # 50m est une valeur completement aléatoire !
                        F = E.destination( 50, self.bearings[0])
                        new_point_bis = intersect_points_bearings_latlon(tmp_point, self.bearings[i-1], E, self.bearings[0])

                    #on croise la parralle au segment[i+1] avant la parralle au segment[i] donc le new_point doit etre le new_point_bis.
                    # il faut verifier que new_point et new_point_bis ont le meme bearing sinon les points ne sont pas dans le bon ordre 
                    #if tmp_point.distanceTo(new_point) >tmp_point.distanceTo(new_point_bis) and tmp_point.compassAngleTo(new_point) == tmp_point.compassAngleTo((new_point_bis)):
                        #pass

                    #print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))

                    # If new_point_bis is closer to tmp_point and inside [tmp_point new_point] segmnet, then the new point is new_point_bis
                    if tmp_point.distanceTo(new_point) >tmp_point.distanceTo(new_point_bis)  and abs(tmp_point.compassAngleTo(new_point) - tmp_point.compassAngleTo((new_point_bis))<2):
                    #if tmp_point.distanceTo(new_point) >tmp_point.distanceTo(new_point_bis) :
                        #print('tmp_point.compassAngleTo(new_point) {} tmp_point.compassAngleTo((new_point_bis)) {}'.format(tmp_point.compassAngleTo(new_point) , tmp_point.compassAngleTo((new_point_bis))))
                       # print('tmp_point.distanceTo(new_point) {} tmp_point.distanceTo(new_point_bis){}'.format(tmp_point.distanceTo(new_point),tmp_point.distanceTo(new_point_bis)))
                       # print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))
                       # self.waypoint_list.append(WayPoint(C, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="C"))
                       # self.waypoint_list.append(WayPoint(D, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="D"))
                       # self.waypoint_list.append(WayPoint(E, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="E"))
                       # self.waypoint_list.append(WayPoint(F, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="F"))
                       # self.waypoint_list.append(WayPoint(tmp_point, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="tmp_point"))
                       # self.waypoint_list.append(WayPoint(new_point, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="new_point"))
                       # self.waypoint_list.append(WayPoint(new_point_bis, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="new_point_bis"))
                        #from IPython import embed; embed()
                        #finish = True
                        #break
                        new_point=new_point_bis


                        # We have a lock with current point
                        lock_points[i]=True

                    


                    # Si le nouveau point n'est pas dirigé dans le sens de angle_bearing, c'est que l'on est allé trop loin
                    new_bearing = tmp_point.compassAngleTo(new_point)
                    angle_bearing=self.bearings[i-1]

                    print('bearing tmp to new {}  bearing angle {}'.format(new_bearing,angle_bearing))
                    
                    if abs(new_bearing-angle_bearing) >1:
                        print('changement de sens donc on arrete ici')
                        print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))

                        finish = True
                        break

                    if j >  78 :
                        print('tmp_point.compassAngleTo(new_point) {} tmp_point.compassAngleTo((new_point_bis)) {}'.format(tmp_point.compassAngleTo(new_point) , tmp_point.compassAngleTo((new_point_bis))))
                        print('tmp_point.distanceTo(new_point) {} tmp_point.distanceTo(new_point_bis){}'.format(tmp_point.distanceTo(new_point),tmp_point.distanceTo(new_point_bis)))
                        print(' tmp_point {} {} new_point {} {} new_point_bis {} {}'.format(tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon, new_point_bis.lat, new_point_bis.lon))
                        self.waypoint_list.append(WayPoint(C, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="C"))
                        self.waypoint_list.append(WayPoint(D, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="D"))
                        self.waypoint_list.append(WayPoint(E, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="E"))
                        self.waypoint_list.append(WayPoint(F, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="F"))
                        self.waypoint_list.append(WayPoint(tmp_point, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="tmp_point"))
                        self.waypoint_list.append(WayPoint(new_point, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="new_point"))
                        self.waypoint_list.append(WayPoint(new_point_bis, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="new_point_bis"))
                        #from IPython import embed; embed()
                        #finish = True
                        break 

                    # on cherche si le nouveau segment coupe un des précédants sans que ce soit le dernier et en partant en arrière
#                    intersec_point, segment = intersect_segments_list([tmp_point, new_point], segments_list[:-1])
                    intersec_point, segment = intersect_segments_list([tmp_point, new_point], segments_list[:-1])
                    intersec_point_CD, segment_CD = intersect_segments_list([tmp_point, new_point], CD_segments_list)
                    
                    if intersec_point_CD:
                        print('intersection CD ici {} {}'.format(intersec_point_CD.lat,intersec_point_CD.lon))

                    ## probleme ici. new_point a peu de chance d'etre sur un ancien CD...
#                    if intersec_point and ( onSegment(segment[0],new_point,segment[1] )):
                    if intersec_point :
                        print('Fini intersection avec segment déja créé')
                        print('Point {} {} C {} {}'.format(point.lat, point.lon, C.lat, C.lon))
                        print('intersc {} {} tmp_point {} {} new_point {} {}'.format(intersec_point.lat, intersec_point.lon, tmp_point.lat, tmp_point.lon, new_point.lat, new_point.lon))
                        print('intersc_segment {} {} {} {}'.format(segment[0].lat, segment[0].lon, segment[1].lat, segment[1].lon))
                        

                        # il faut trouver le dernier point

                        #le nouveau point coupe la parralle a un vieux segment. Il faut s'arretera l'intersection 
                        if not onSegment(segment[0],new_point,segment[1]):
                            print('le nouveau point coupe la parralle a un vieux segment')
                            self.waypoint_list.append(WayPoint(intersec_point, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="Last"))
                            finish = True
                            break

                        # on teste si la droite parralelle au segment coupé passant par tmp_point est plus éloigné de self.increment_lat. si ce n'est pas le cas alors
                        # tmp est le dernier point
                        # on trouve l'angle entre le segment coupé et [tmp,new]

                        angle = getAnglelatlon(segment[1], intersec_point, tmp_point)
                        H = tmp_point.distanceTo(intersec_point)*sin(radians(angle))
                        #print('H {} self.increment_lat {}'.format(H, self.increment_lat))

                        if H <= self.increment_lat:
                            print('tmp est le dernier point possible')
                            finish = True
                            break
                        # sinon, on trouve l'intersection avec la parrallele au segment décalé de self.increment_lat
                        distance = (H-self.increment_lat)/sin(radians(angle))
                        # Le +90 depend si on est clockwise !!!!
                        last_point = tmp_point.destination(distance, tmp_point.compassAngleTo(new_point))
                        self.waypoint_list.append(WayPoint(last_point, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale, wp_text="Last"))
                        finish = True
                        break
                    self.waypoint_list.append(WayPoint(
                        new_point, self.bearings[i], emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
                segments_list.append([tmp_point, new_point])
                CD_segments_list.append([C, D])
                tmp_point = new_point
                i += 1

            round_nb += 1
            finish_condition=all(lock_points)

#

    def generate_path_normal(self):
        """ path type allez-retour
        avec des parrallelles"""
        # point de départ
        indice_gauche = self.nb_points
        indice_droit = 1
        right = True
        limite_AB = .5
        finish = False
        not_in_right = False
        not_in_left = False

        tmp_A = self.points[0]
        tmp_B = self.points[1]
        # la direction initiale est celle de points[0] vers points[1]
        direction_right = tmp_A.compassAngleTo(tmp_B)
        direction_left = tmp_B.compassAngleTo(tmp_A)

        tmp = None
        # premiers point
        self.waypoint_list.append(WayPoint(
            tmp_A, direction_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))

        self.waypoint_list.append(WayPoint(tmp_B, tmp_B.compassAngleTo(self.points[indice_droit+1]),
                                           emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale))
        i = 0

        # on s'arrete quand cette distance est très petite
        while (tmp_A.distanceTo(tmp_B) > limite_AB) and not finish:
            #print('i {}'.format(i))
            # if i == 15:
            #    break
            direction_ext_right = tmp_B.compassAngleTo(
                self.points[indice_droit+1])
            direction_ext_left = tmp_A.compassAngleTo(
                self.points[indice_gauche-1])

            # Le point C est  sur la parralle à [tmp_A tmp_B]
            C = tmp_A.destination(self.increment_lat, direction_right+self.othogonal_increment)
            # le point D est sur la ligne C avec son bearing et à distance d=10m sans importance
            D = C.destination(10, direction_left)

            # on trouve les points d'intersection de C avec son bearing et les segments latéraux
            # self.waypoint_list.append(WayPoint(
            #        C, direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale,text="C"))



            intersect_right = intersect_four_points_latlon(
                C, D, tmp_B, self.points[indice_droit+1])

            # verification que les points son bien sur les segments
            if not iswithinLatLontTri(intersect_right, tmp_B, self.points[indice_droit+1]):
                #not_in_right = True
                if debug:
                    print('not in segment right')
                indice_droit += 1
                # Arret si le nouveau sommet est celui du coté opposé
                if (indice_droit == indice_gauche) or (indice_droit > self.nb_points - 2):
                    if debug:
                        print('finish')
                    finish = True
                    break
                direction_ext_right = self.points[indice_droit].compassAngleTo(
                    self.points[indice_droit+1])

                intersect_right = intersect_four_points_latlon(
                    C, D, self.points[indice_droit], self.points[indice_droit+1])

                if not iswithinLatLontTri(intersect_right, self.points[indice_droit], self.points[indice_droit+1]):
                    if debug:
                        print('Really not in segment right !')
                    not_in_right = True

            intersect_left = intersect_four_points_latlon(
                C, D, tmp_A, self.points[indice_gauche-1])
            # si il n'y a pas d'intersection dans ce segment, on prend le suivant
            if not iswithinLatLontTri(intersect_left, tmp_A, self.points[indice_gauche-1]):
                if debug:
                    print('not in segment left')
                indice_gauche -= 1
                # Arret si le nouveau sommet est celui du coté opposé
                if (indice_droit == indice_gauche):
                    if debug:
                        print('finish')
                    finish = True
                    break
                direction_ext_left = self.points[indice_gauche].compassAngleT(
                    self.points[indice_gauche-1])

                intersect_left = intersect_four_points_latlon(
                    C, D, self.points[indice_gauche], self.points[indice_gauche-1])

                if not iswithinLatLontTri(intersect_left, self.points[indice_gauche], self.points[indice_gauche-1]):
                    if debug:
                        print('Really not in segment left !')
                    not_in_left = True

            if not_in_left and not_in_right:
                if debug:
                    print('Finish !!!! ')
                finish = True
                # self.waypoint_list.append(WayPoint(
                #    C, direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale,text="C"))
                #self.waypoint_list.append(WayPoint(D                    , direction_ext_right, emprise_laterale=self.emprise_laterale, emprise_longitudinale=self.emprise_longitudinale,text="D"))

                break
            # arret en cas de pb
#            if(intersect_right[0] < 0) or intersect_left[0] < 0:
            if(intersect_right.lat < 0) or intersect_left.lon < 0:
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

        total_distance = 0.0
        nb_turns = 0

        # start point
        tmp_point = self.start_point

        for waypoint in self.waypoint_list:
            total_distance += tmp_point.distanceTo(waypoint.point)
            tmp_point= waypoint.point
            nb_turns +=1


        print('############################################################')
        print('{} Total distance is {}m with {} turns'.format(self.style, total_distance, nb_turns))
        print('############################################################')

    def export_to_kml(self):
        """ Export waypoints to kml for DJI UAV"""
        pass


def main(args):
    """la fonction main"""

    """ IPGP
    start_point_list = []
    start_point_list.append(LatLon(48.846383084057315, 2.356424447320463))
    start_point_list.append(LatLon(48.84415899803569, 2.353495475053588))
    start_point_list.append(LatLon(48.844599153918324, 2.355340339361402))
    start_point_list.append(LatLon(48.84508549389749, 2.356311190101862))

    a = LatLon(48.844781966005414, 2.354806246580006)
    b = LatLon(48.845476490908986, 2.3559582742434224)
    c = LatLon(48.844800522139515, 2.356945151087957)
    d = LatLon(48.84415592294359, 2.3565687535257593)
    e = LatLon(48.84395753653702, 2.355015706155173)
    f = LatLon(48.844565798460536, 2.3552507666007094)
    """

    """
    Martinique
    """
    start_point = LatLon(14.810500300074992, -61.1768933155967)
    a = LatLon(14.803299290149893, -61.178222440791735)
    b= LatLon(14.801850020555767, -61.17780919765657)
    c= LatLon(14.801301645750753, -61.176334486860505)
    d= LatLon(14.803784989143798, -61.17344988772093)
    e= LatLon(14.804842555809342, -61.17457617783481)
    f= LatLon(14.805179409280854, -61.176974608580075)

#    points = deque([f, e, d, c, b, a])
#    points = deque([a, b, c])
#    points = deque([a, b, c, d])
#    points = deque([d,c,b,a])
#    points = deque([a, b, c, d, e])
    points = deque([a,b,c,d,e,f])

    points.rotate(-1)

    emprise_laterale = 80
    emprise_longitudinale = 56



#    emprise_laterale = 40
#    emprise_longitudinale = 35
    # on fait varier les start_point
    #    for start_point in start_point_list:

    # on fait varier le point de départ
#    for i in range(len(points)):


    Path_generator_snail = PathPlanning(points=points,  emprise_laterale=emprise_laterale,
                                  emprise_longitudinale=emprise_longitudinale, start_point=start_point, recouvrement_lat=0.1, recouvrement_lon=0.10)
   
#    Path_generator_snail.generate_path("snail")
    Path_generator_snail.generate_path("espiral")

#    the_map = WaypointMap(start_point)
    the_map_snail = WaypointMap()
    # on place les limites de la zone
    the_map_snail.add_polygon(points=points, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")
    # On ajoute les waypoint qui ont été trouvés a la carte
    for wp in Path_generator_snail.waypoint_list:
        the_map_snail.add_waypoint(wp, direction=False, footprint=True,footprint_markers=False)
        
    the_map_snail.add_path_waypoints(Path_generator_snail.waypoint_list)
    # Exportation de la carte
    the_map_snail.export_to_file('snail')

    #print('Start point is {}'.format(points[0]))
    # Path_generator.export_to_kml()
    Path_generator_snail.compute_length_and_turns()

    wp_list =[]
    for wp in Path_generator_snail.waypoint_list:
        wp_list.append(wp.latlon())

   # from IPython import embed; embed()

#    print(wp_list)

    # on fait tourner le 1er point
    # points.rotate(1)
if __name__ == '__main__':
    main(sys.argv)
