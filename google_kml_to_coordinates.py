#!/usr/bin/env python3
import argparse

parser = argparse.ArgumentParser(description='Process Google Kml to coordinates for .ini file')
parser.add_argument('--infile', '-i',type=argparse.FileType('r', encoding='UTF-8'), 
                      required=True)
args = parser.parse_args()

debug = False
next_line_is_coordinates = False

while line := args.infile.readline():
    if "<coordinates>" in line:
        next_line_is_coordinates = True

    if next_line_is_coordinates:
        line = args.infile.readline()
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
