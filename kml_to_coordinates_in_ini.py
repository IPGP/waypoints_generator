#!/usr/bin/env python3
import argparse
import os
import sys
from tempfile import mkstemp
from shutil import move, copymode
from lxml import etree


parser = argparse.ArgumentParser(
    description='Process Google Kml coordinates from Google map or Clearance into .ini file')
parser.add_argument('--in_kml_file', '-i', type=argparse.FileType('r', encoding='UTF-8'),
                    required=True, help='Kml file to be processed')
parser.add_argument('--out_ini_file', '-o', required=False,
                    help='ini file to be modified with kml data')

args = parser.parse_args()

DEBUG = False

tree = etree.parse(args.in_kml_file)
root = tree.getroot()
raw_coordinates = root.find('.//coordinates',root.nsmap).text 

#Clean string from google
# ie remove \n and \t and maybe trailing space
coordinates_string = raw_coordinates.replace('\n','').replace('\t','').strip()

#from IPython import embed; embed()

coordinates_tab = coordinates_string.split(" ")
nb_of_coordinates = len(coordinates_tab)


nb_of_points=0
final_string = 'points_list =['


# -1 because the last point of the Polygon is also the first point
for i in range(nb_of_coordinates-1):
    nb_of_points+=1

    coordinate = coordinates_tab[i]
    lon, lat, alt = coordinate.split(',')
    if DEBUG:
        print(f'lat {lat} lon {lon} alt {alt}')
    final_string += f'({lat},{lon}),'

if nb_of_points == 0:
    print(F'No points found in file : {args.in_kml_file.name}')
    sys.exit(-1)
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
