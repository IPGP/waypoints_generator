#!/usr/bin/env python 

debug = False

inputname = './Noisiel Mapping.kml'

next_line_is_coordinates = False

# Read KML file and get coordinates
with open(inputname) as kml_file:
     while line := kml_file.readline():
        #print(line.rstrip())        
        if "<coordinates>" in line:
            next_line_is_coordinates = True
        
        if next_line_is_coordinates:
            line = kml_file.readline()
            coordinates = line
            next_line_is_coordinates = False

coordinates_string=coordinates.lstrip().rstrip()

final_string='points_list =['
for coordinate in coordinates_string.split(" "):
    lon,lat, alt = coordinate.split(',')
    if debug : print(f'lat {lat} lon {lon} alt {alt}')
    final_string += f'({lat},{lon}),'

# end of coordinates

# strip last coma
final_string = final_string[:-1]
# add parenthesis
final_string += ']'

print(final_string)
