from waypointsmap import WaypointMap
from waypoint import WayPoint
from pathplanning import PathPlanning
from pygeodesy.sphericalTrigonometry import LatLon
from collections import deque

"""
Martinique
"""
start_point = LatLon(14.810431373395028, -61.176753253004485)
start_point.text = 'Start'
a = LatLon(14.801717517497558, -61.17830598375636)
b = LatLon(14.803184637915658, -61.177470922032136)
c = LatLon(14.804110982101069, -61.176259107347896)
d = LatLon(14.80476281073443, -61.175343978459765)
e = LatLon(14.804147551878703, -61.17414211429372)
f = LatLon(14.802389075700889, -61.175630772903205)
g = LatLon(14.801758424759862, -61.176496729696545)

#    points = deque([f, e, d, c, b, a])
#    points = deque([a,f, e, d, c, b ])
#    points = deque([a, b, c])
#    points = deque([a, b, c, d])
points = deque([a, b, c, d, e, f, g])

lateral_footprint = 80

the_map = WaypointMap()
the_map.add_area_of_interest(AOI)
pathplanning = PathPlanning(AOI)
the_map.add_path(pathplanning)

IPGP = Waypoint(latitude=LAT,  longitude=LON, name="Center", direction=30)

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
