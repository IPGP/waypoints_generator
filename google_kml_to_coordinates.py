#!/usr/bin/env python3
import argparse
import os
from tempfile import mkstemp
from shutil import move, copymode


parser = argparse.ArgumentParser(
    description='Process Google Kml coordinates from Google map or Clearance into .ini file')
parser.add_argument('--in_kml_file', '-i', type=argparse.FileType('r', encoding='UTF-8'),
                    required=True, help='Kml file to be processed')
parser.add_argument('--out_ini_file', '-o', required=False,
                    help='ini file to be modified with kml data')

args = parser.parse_args()

DEBUG = False
NEXT_LINE_IS_COORDINATES = False

while line := args.in_kml_file.readline():
    if "<coordinates>" in line:
        NEXT_LINE_IS_COORDINATES = True

    if NEXT_LINE_IS_COORDINATES:
        line = args.in_kml_file.readline()
        coordinates = line
        NEXT_LINE_IS_COORDINATES = False

# removing leading and trailing spaces
coordinates_string = coordinates.lstrip().rstrip()

# split each point
coordinates_tab = coordinates_string.split(" ")
nb_of_coordinates = len(coordinates_tab)

final_string = 'points_list =['
for i in range(nb_of_coordinates-1):
    coordinate = coordinates_tab[i]
    lon, lat, alt = coordinate.split(',')
    if DEBUG:
        print(f'lat {lat} lon {lon} alt {alt}')
    final_string += f'({lat},{lon}),'

# end of coordinates
# strip last coma
final_string = final_string[:-1]
# add parenthesis
final_string += ']\n'

print(final_string)

# Write to .ini file
if args.out_ini_file:
    # Creating a temp file
    fd, abspath = mkstemp()
    with os.fdopen(fd, 'w') as tmp_file:
        with open(args.out_ini_file, 'r', encoding='utf-8') as file0:
            for line in file0:
                if not "points_list =" in line:
                    tmp_file.write(line)
                else:
                    tmp_file.write(final_string)

    copymode(args.out_ini_file, abspath)
    os.remove(args.out_ini_file)
    move(abspath, args.out_ini_file)
