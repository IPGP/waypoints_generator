import folium
from gpxplotter import create_folium_map


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

    def add_wapypoint(self, location, color, popup_text="", icon='fa-map-pin'):
        folium.Marker(location=location, popup=popup_text,
                      icon=folium.Icon(color=color, icon=icon)).add_to(self.the_map)

    def add_polygon(self, locations, color, fill_color, fill_opacity, weight, popup, fill=True):
        self.the_map.add_child(folium.vector_layers.Polygon(locations=locations, color=color, fill=True,
                                                            fill_color=fill_color, fill_opacity=fill_opacity, weight=2, popup=popup))

    def add_area_of_interest(self, locations):
        self.the_map.add_child(folium.vector_layers.Polygon(locations=locations, color='#348feb', fill=True,
                                                            fill_color='#34e5eb', fill_opacity=0.3, weight=2, popup=""))

    def export_to_file(self, filename='./carte.html'):
        self.the_map.fit_bounds(self.the_map.get_bounds(), padding=(5, 5))
        folium.LayerControl(sortLayers=True).add_to(self.the_map)
        self.the_map.save(filename)

    def add_path(self, pathplanning):
        pass
