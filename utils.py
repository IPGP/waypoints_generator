#!/usr/bin/python
# -*- coding:utf-8 -*-
from geopy.distance import geodesic
from math import degrees, acos, radians
from geographiclib.geodesic import Geodesic
from geopy import Point, distance, point
import geopy
from math import degrees, sqrt, sin, cos
import sys

from numpy import result_type
from pygeodesy.sphericalTrigonometry import LatLon as latlontri
from pygeodesy.sphericalNvector import *

debug = False
debug = True



def getAngle(A, B, C):
    "return angle en degrees between A,B and C in degrees"
    AB = geodesic(A, B).m
    BC = geodesic(B, C).m
    CA = geodesic(A, C).m

    return degrees(acos((AB*AB+BC*BC-CA*CA)/(2*AB*BC)))


def getDistance(A, B):
    return geodesic(A, B).m


def getBearing(A, B):
    "return bearing en degrees between A and B"
    return Geodesic.WGS84.Inverse(A[0], A[1], B[0], B[1])['azi1']


def point_distance_bearing_to_new_point(point, distance, bearing):
    """return new coordinates of the input point with distance and bearing
        distance in meters
    """

    dist = geopy.distance.distance(meters=(distance))
    tmp = dist.destination(point=Point(point[0], point[1]), bearing=bearing)
#
    # print("angle "+str(angle + self.orientation))
    return [tmp.latitude, tmp.longitude]

def middlepoint(A,B):
    """ return the point in the middle of AB segment"""
    a = LatLon(A[0], A[1])
    b = LatLon(B[0], B[1])
    c=a.intermediateTo(b,.5)
    return [c.lat,c.lon ] 


def iswithin(A, point1, point2):
    """
    Check whether this point is between two other points.
    If this point is not on the great circle arc defined by both points,
    return whether it is within the area bound by perpendiculars to the great
    circle at each point (in the same hemispere).
    """
    a = LatLon(A[0], A[1])
    b = LatLon(point1[0], point1[1])
    c = LatLon(point2[0], point2[1])
    list = b, c
#    return a.isenclosedBy(list)
    return a.iswithin(b, c)


def intersect_four_points_(A,B,C,D):
    """
    return the intersection point of the line AB and line CD
    """

    a = latlontri(A[0], A[1])
    b = latlontri(B[0], B[1])
    c = latlontri(C[0], C[1])
    d = latlontri(D[0], D[1])

    c = a.intersection( b, c,d)
    # probleme d'antipode. Si le point d'intersection est à plus de 10000km, il y a un probleme
    if (c.distanceTo(a) > 10000)and (c.distanceTo(a) > 10000):
        c= c.antipode()

    return [c.lat, c.lon]


def intersect_points_bearings(A, bearing_A, B, bearing_B):
    """ return the intersection point C of the two lines
    from A with bearing_A to B with bearing_B
    """
    a = LatLon(A[0], A[1])
    b = LatLon(B[0], B[1])

    c = a.intersection(bearing_A, b, bearing_B)
        
    return [c.lat, c.lon]


def pouet():
    pass


def main():
    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84415592294359, 2.3565687535257593)
    E = (48.84395753653702, 2.355015706155173)

#    print(getAngle(A, B, C))
    #   print(getBearing(A, B))
  #  print(getBearing(B, C))
    A_to_B_bearing = getBearing(A, B)
    C_to_B_bearing = getBearing(C, B)

    inter = intersect_points_bearings(A, A_to_B_bearing, C, C_to_B_bearing)
    print('B is {}, B intersect is {} difference {}'.format(
        B, inter, getDistance(B, inter)))
    iswithin(A, B, C)


if __name__ == '__main__':
    main()
