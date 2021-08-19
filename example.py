from waypointsmap import WaypointMap
from waypoint import Waypoint


LAT = 48.84482270388685
LON = 2.3562098704389163

X0 = 48.84643250706535
Y0 = 2.3527444567404943
X1 = 48.84350234422499
Y1 = 2.358956452357875
# Area Of Interest
AOI = [(X0, Y0),  (X0, Y1), (X1, Y1), (X1, Y0)]

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
