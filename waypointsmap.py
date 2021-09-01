import folium
from gpxplotter import create_folium_map
from waypoint import WayPoint
from utils import getAngle

# https://fontawesome.com/v4.7/icons/


class WaypointMap:
    "WaypointMap class to plot waypoints of a mission in a nice map"

    def __init__(self):
        self.the_map = create_folium_map(
            zoom_min=0, max_zoom=18, zoom_start=10)

        # Add custom base maps to folium
        basemaps = {
            'Google Maps': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Maps',
                overlay=False,
                control=True
            ),
            'Google Satellite': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Satellite',
                overlay=True,
                control=True
            ),
            'Google Terrain': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=p&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Terrain',
                overlay=True,
                control=True
            ),
            'Google Satellite Hybrid': folium.TileLayer(
                tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
                attr='Google',
                name='Google Satellite',
                overlay=True,
                control=True
            ),
            'Esri Satellite': folium.TileLayer(
                tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                attr='Esri',
                name='Esri Satellite',
                overlay=True,
                control=True
            )
        }

        # Add custom basemaps
        basemaps['Google Maps'].add_to(self.the_map)
        basemaps['Google Satellite'].add_to(self.the_map)
        basemaps['Google Terrain'].add_to(self.the_map)
        basemaps['Google Satellite Hybrid'].add_to(self.the_map)
        basemaps['Esri Satellite'].add_to(self.the_map)

        tiles_maps = ['openstreetmap', 'Stamen Terrain']
        # tiles_maps = ['openstreetmap']
        # tiles_maps = []
        # tiles_maps=[ 'openstreetmap',''Cartodb Positron',
        # 'Stamen Terrain','Stamen Toner','Stamen Watercolor']

        for tile in tiles_maps:
            folium.TileLayer(tile).add_to(self.the_map)

    def add_waypoint(self, waypoint, color='blue', popup_text=""):
        # print(waypoint.orientation)
        folium.Marker(location=waypoint.location, popup=popup_text, icon=folium.Icon(
            color='beige', angle=int(waypoint.orientation), icon='arrow-up', prefix='fa')).add_to(self.the_map)

        folium.Marker(location=waypoint.X0, popup='X0', icon=folium.Icon(
            color='red', icon='fa-map-pin')).add_to(self.the_map)

        folium.Marker(location=waypoint.X1, popup='X0', icon=folium.Icon(
            color='green', icon='fa-map-pin')).add_to(self.the_map)

        folium.Marker(location=waypoint.X2, popup='X0', icon=folium.Icon(
            color='green', icon='fa-map-pin')).add_to(self.the_map)

        folium.Marker(location=waypoint.X3, popup='X0', icon=folium.Icon(
            color='red', icon='fa-map-pin')).add_to(self.the_map)

        self.the_map.add_child(folium.vector_layers.Polygon(locations=[waypoint.X0, waypoint.X1, waypoint.X2, waypoint.X3],
                                                            color='blue', fill=True,
                                                            fill_color='blue', fill_opacity=0.3, weight=2, popup=""))

    def add_polygon(self, locations, color, fill_color, fill_opacity, weight, popup, fill=True):
        """ Plot shape of the mapping area with dots"""
        self.the_map.add_child(folium.vector_layers.Polygon(locations=locations, color=color, fill=True,
                                                            fill_color=fill_color, fill_opacity=fill_opacity, weight=2, popup=popup))
        nb = 0
        for point in locations:
            folium.Marker(location=point, tooltip=str(nb)+"<br>"+str(point[0])+"<br>"+str(point[1]),
                          icon=folium.Icon(color='blue', icon="circle", prefix='fa')).add_to(self.the_map)
            nb += 1

    def add_area_of_interest(self, locations):
        self.the_map.add_child(folium.vector_layers.Polygon(locations=locations, color='#348feb', fill=True,
                                                            fill_color='#34e5eb', fill_opacity=0.3, weight=2, popup=""))

    def export_to_file(self, filename='./carte.html'):
        self.the_map.fit_bounds(self.the_map.get_bounds(), padding=(5, 5))
        folium.LayerControl(sortLayers=True).add_to(self.the_map)
        self.the_map.save(filename)

    def add_path(self, pathplanning):
        pass


def main():

    A = (48.844781966005414, 2.354806246580006)
    B = (48.845476490908986, 2.3559582742434224)
    C = (48.844800522139515, 2.356945151087957)
    D = (48.84415592294359, 2.3565687535257593)
    E = (48.84395753653702, 2.355015706155173)

    zone = [A, B, C, D, E]

    emprise_laterale = 40
    emprise_longitudinale = 25

    the_map = WaypointMap()

    the_map.add_polygon(locations=zone, color='#ff7800', fill=True,
                        fill_color='#ffff00', fill_opacity=0.2, weight=2, popup="")

    lat = 48.84482270388685
    lon = 2.3562098704389163

    orientation = getAngle(A, B, C)
    loc_IPGP = [lat, lon]
    IPGP = WayPoint(loc_IPGP, orientation, emprise_laterale,
                    emprise_longitudinale)

    the_map.add_waypoint(IPGP)
    the_map.export_to_file()


if __name__ == '__main__':
    main()
