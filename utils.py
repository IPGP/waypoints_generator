#!/usr/bin/python
# -*- coding:utf-8 -*-
from geopy.distance import geodesic
from math import degrees, acos
from geographiclib.geodesic import Geodesic
from geopy import Point, distance
import geopy
from math import  degrees, sqrt


def getAngle(A, B, C):
    "return angle en degrees between A,B and C in degrees"
    AB = geodesic(A, B).m
    BC = geodesic(B, C).m
    CA = geodesic(A, C).m

    return degrees(acos((AB*AB+BC*BC-CA*CA)/(2*AB*BC)))


def getBearing(A, B):
    "return bearing en degrees between A and B"
    return Geodesic.WGS84.Inverse(A[0], A[1], B[0], B[1])['azi1']

def point_distance_bearing_to_new_point(point,distance,bearing):
    """return new coordinates of the input point with distance and bearing
        distance in meters
    """

    dist = geopy.distance.distance(meters=(distance))
    tmp = dist.destination(point=Point(point[0],point[1]), bearing=bearing)
#
    #print("angle "+str(angle + self.orientation))
    return  [tmp.latitude, tmp.longitude]

def main():
    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84415592294359, 2.3565687535257593)
    E = (48.84395753653702, 2.355015706155173)

    print(getAngle(A, B, C))
    print(getBearing(A, B))
    print(getBearing(B, C))


if __name__ == '__main__':
    main()
