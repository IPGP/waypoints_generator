#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
from math import tan, radians, degrees, atan, cos, ceil, sqrt, floor, pi, sin
from PIL import Image as ImagePil
import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib
matplotlib.use('svg')
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.patches as mpatches
import matplotlib.image as mpimg
import pprint

# Defines the successive positions (orientations) of the drone along one path,
# to acquire images with a defined overlap ratio, from one image to the next.
# In addition, the gimbal pitch is adapted, so that photos are taken parallel to
# the ground.
# Copyright (C) 2021 Arthur Delorme - v0.9

# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <http://www.gnu.org/licenses/>.
#
# (contact: delorme@ipgp.fr)

class DroneOri(object):
    """
    Defines the successive positions (orientations) of the drone along one path,
    to acquire images with a defined overlap ratio, from one image to the next.
    In addition, the gimbal pitch is adapted, so that photos are taken parallel
    to the ground.
    
    Methods:
        __init__
        read_dsm            Reads the DSM data
        compute_prof_az     Computes the azimuth of the profile
        compute_fov         Calculates the camera field of view, in the
                                longitudinal direction
        compute_footprint   Computes the footprint of an image, based on h and
                                fov_lon
        compute_gsd         Calculates the images ground sampling distance (GSD)
        read_tfw            Reads georeferencing information from the DSM .tfw
                                file
        dsm_profile         Makes a profile in the DSM
        find_ori            Finds the gimbal pitch angle of the drone and its
                                azimuth for the current position
        apply_height        Applies the desired flight distance at the position
                                found by find_ori(), taking the pitch into
                                account
        add_ori             Adds an orientation along the profile, at the given
                                index
        estim_overlp        Estimates the overlap ratio between two images
        estim_b_on_h        Estimates the B/H ratio between two images
        insert_ori          Inserts an orientation between two orientations
        drone_orientations  Estimates the drone orientation at each shot, from
                                the profile extracted from the DSM
        draw_orientations   Draws the DSM profile with the drone orientations
        draw_map            Draws the path with all orientations on a map
        export_ori          Exports all orientation information
    """
    
    def __init__(self, name, dsm, tfw, a_east, a_north, b_east, b_north,
            h, sensor_size, img_size, focal, ovlp, ref_alti=0, takeoff_pt=None,
            fov_lon=None, fov_lat=None, footprint=None, profile=None,
            prof_az=None, x_spacing=None, y_spacing=None, top_left_e=None,
            top_left_n=None, dsm_profile_margin=None, drone_ori=None,
            ovlp_linreg_x=None, ovlp_linreg_z=None, ovlp_linreg_stats=None,
            final_overlap=None, fixed_pitch=None):
        """
        Variables:
            name                [string] name of the drone path
            dsm                 [string] path to the DSM .tif file;
                                    the unit must be the meter, for each
                                    dimension (i.e. not in the WGS84 reference
                                    system)
            tfw                 [string] path to the DSM .tfw file
            a_east, a_north     [floats] Easting and Northing coordinates of the
                                    first point of the profile in the DSM (in m)
            b_east, b_north     [floats] Easting and Northing coordinates of the
                                    last point of the profile in the DSM (in m)
            ref_alti            [float] reference altitude (in m, default is 0),
                                    which will be subtracted to the z coordinate
                                    of the drone;
                                    it can be either 0, to keep the altitude
                                    value as z, or another altitude (e.g. the
                                    drone take off altitude), to ensure that z
                                    is the height with respect to that altitude;
                                    if used along with takeoff_pt, an error will
                                    be raised
            takeoff_pt         [tuple] Easting and Norting coordinates (e, n)
                                    of the drone takeoff point;
                                    if used along with ref_alti, an error will
                                    be raised.
            h                   [float] flight distance, i.e. distance between
                                    the drone and the ground (in m)
            sensor_size         [tuple] size of the sensor (L,l) (in mm)
            img_size            [tuple] size of the image (L,l) (in px)
            focal               [float] focal length of the sensor (in mm)
            ovlp                [float] overlap ratio between two consecutive
                                    images (shoud be between 0.5 and 0.95)
            fov_lon, fov_lat    [float] camera field of view in the longitudinal
                                    and lateral directions (in rad)
            footprint           [float] image footprint (in m)
            profile             [list] profile between a and b in the DSM (e, n,
                                    z)
            prof_az             [float] azimuth of the profile (in rad)
            x_ and y_spacing    [float] DSM GSD (in m)
            top_left_e and _n   [float] Easting and Northing coordinates of the
                                    top-left corner of the DSM top-left pixel
                                    (in m)
            dsm_profile_margin  [int] margin to cover both endpoints of the
                                    profile (w/o units);
                                    equals to: ceil(footprint/2/x_spacing) + 1;
                                    (i.e. nb of extra values that were added to
                                    each end of self._profile, +1 by safety)
            drone_ori           [list] drone orientation (see the add_ori()
                                    method)
            ovlp_linreg_x       [dict] x values of the linear regressions found
                                    in estim_ovlp()
            ovlp_linreg_z       [dict] z values of the linear regressions found
                                    in estim_ovlp()
            ovlp_linreg_stats   [dict] values and stats of the linear
                                    regressions found in estim_ovlp()
            final_overlap       [dict] overlap ratios for each pair of
                                    orientations
            fixed_pitch         [float] gimbal pitch value (in deg);
                                    1) if given, pitch will be fixed instead of
                                    being estimated for each orientation;
                                    2) pitch should always be negative (-90 <=
                                    fixed_pitch < 0). Here, by convention, a
                                    positive pitch means that we want the drone
                                    to shoot backwards, instead of forwards.
        """
        
        self._name = name
        self._dsm = dsm
        self._np_dsm = None
        self._tfw = tfw
        self._a_east = a_east
        self._a_north = a_north
        self._b_east = b_east
        self._b_north = b_north
        self._ref_alti = ref_alti
        self._takeoff_pt = takeoff_pt
        self._h = h
        self._sensor_size = sensor_size
        self._img_size = img_size
        self._focal = focal
        self._ovlp = ovlp
        self._fov_lon = fov_lon
        self._fov_lat = fov_lat
        self._footprint = footprint
        self._profile = profile
        self._prof_az = prof_az
        self._x_spacing = x_spacing
        self._y_spacing = y_spacing
        self._top_left_e = top_left_e
        self._top_left_n = top_left_n
        self._dsm_profile_margin = dsm_profile_margin
        self._drone_ori = drone_ori
        self._ovlp_linreg_x = ovlp_linreg_x
        self._ovlp_linreg_z = ovlp_linreg_z
        self._ovlp_linreg_stats = ovlp_linreg_stats
        self._final_overlap = final_overlap
        self._fixed_pitch = fixed_pitch
        
        if self._ref_alti is not 0:
            if self._takeoff_pt:
                sys.exit("Error: ref_alti and takeoff_pt are incompatible "
                    "option. Please use only one at a time.")
            else:
                print("Be careful when providing ref_alti: its vertical "
                    "reference must be the same as that of the DSM (e.g. the "
                    "ellipsoid or the geoid)")
        
        self.read_dsm()
        self.compute_prof_az()
        self.compute_fov()
        self.compute_footprint()
        self.compute_gsd()
        self.read_tfw()
        if self._takeoff_pt:
            self.ref_alti_from_takeoff_pt()
    
    @property
    def name(self):
        return self._name
    
    @property
    def dsm(self):
        return self._dsm
    
    @property
    def np_dsm(self):
        return self._np_dsm
    
    @property
    def tfw(self):
        return self._tfw
    
    @property
    def a_east(self):
        return self._a_east
    
    @property
    def a_north(self):
        return self._a_north
    
    @property
    def b_east(self):
        return self._b_east
    
    @property
    def b_north(self):
        return self._b_north
    
    @property
    def ref_alti(self):
        return self._ref_alti
    
    @property
    def takeoff_pt(self):
        return self._takeoff_pt
    
    @property
    def h(self):
        return self._h
    
    @property
    def sensor_size(self):
        return self._sensor_size
    
    @property
    def img_size(self):
        return self._img_size
    
    @property
    def focal(self):
        return self._focal
    
    @property
    def ovlp(self):
        return self._ovlp
    
    @property
    def fov_lon(self):
        return self._fov_lon
    
    @property
    def fov_lat(self):
        return self._fov_lat
    
    @property
    def footprint(self):
        return self._footprint
    
    @property
    def profile(self):
        return self._profile
    
    @property
    def prof_az(self):
        return self._prof_az
    
    @property
    def x_spacing(self):
        return self._x_spacing
    
    @property
    def y_spacing(self):
        return self._y_spacing
    
    @property
    def top_left_e(self):
        return self._top_left_e
    
    @property
    def top_left_n(self):
        return self._top_left_n
    
    @property
    def dsm_profile_margin(self):
        return self._dsm_profile_margin
    
    @property
    def drone_ori(self):
        return self._drone_ori
    
    @property
    def ovlp_linreg_x(self):
        return self._ovlp_linreg_x
    
    @property
    def ovlp_linreg_z(self):
        return self._ovlp_linreg_z
    
    @property
    def ovlp_linreg_stats(self):
        return self._ovlp_linreg_stats
    
    @property
    def final_overlap(self):
        return self._final_overlap
    
    @property
    def fixed_pitch(self):
        return self._fixed_pitch
    
    def read_dsm(self):
        """
        Reads the DSM data
        """
        
        self._np_dsm = np.array(ImagePil.open(self._dsm))
    
    def compute_prof_az(self):
        """
        Computes the azimuth of the profile
        
        0 is the North; positive clockwise
        """
        
        delta_e = self._b_east - self._a_east
        delta_n = self._b_north - self._a_north
        
        if delta_e > 0:
            if delta_n < 0:
                self._prof_az = abs(atan(delta_n / delta_e)) + pi/2
            elif delta_n > 0:
                self._prof_az = abs(atan(delta_e / delta_n))
            else:
                self._prof_az = pi/2
        elif delta_e < 0:
            if delta_n > 0:
                self._prof_az = abs(atan(delta_n / delta_e)) + 3*pi/2
            elif delta_n < 0:
                self._prof_az = abs(atan(delta_e / delta_n)) + pi
            else:
                self._prof_az = 3*pi/2
        else:
            if delta_n > 0:
                self._prof_az = 0
            elif delta_n < 0:
                self._prof_az = pi
            else:
                sys.exit("Error: delta_e = 0 and delta_n = 0")
    
    def compute_fov(self):
        """
        Calculates the camera field of view, in the longitudinal direction
        """
        
        self._fov_lat = 2 * atan(self._sensor_size[0] / (2 * self._focal))
        self._fov_lon = 2 * atan(self._sensor_size[1] / (2 * self._focal))
    
    def compute_footprint(self):
        """
        Computes the footprint of an image, based on h and fov
        """
        
        self._footprint = 2 * self._h * tan(self._fov_lon / 2)
    
    def compute_gsd(self):
        """
        Calculates the images ground sampling distance (GSD)
        """
        
        self._gsd = self._h * self._sensor_size[0] / (self._focal * \
            self._img_size[0])
    
    def read_tfw(self):
        """
        Reads georeferencing information from the DSM .tfw file
        
        Note: 0 is the only supported value for the rotation parameters
        """
        
        with open(self._tfw) as f:
            for i, l in enumerate(f):
                if i == 0:
                    self._x_spacing = float(l)
                elif i == 3:
                    self._y_spacing = float(l)
                elif i == 4:
                    self._top_left_e = float(l)
                elif i == 5:
                    self._top_left_n = float(l)
            self._top_left_e -= self._x_spacing / 2 # Top-left corner of px
            self._top_left_n -= self._y_spacing / 2
            if abs(self._x_spacing) != abs(self._y_spacing):
                print("Warning: abs(x_spacing) != abs(y_spacing).")
    
    def ref_alti_from_takeoff_pt(self):
        """
        Extract ref_alti from the DSM at the coordinates of the takeoff point
        """
        
        row = round((self._takeoff_pt[1] - self._top_left_n) / self._y_spacing)
        col = round((self._takeoff_pt[0] - self._top_left_e) / self._x_spacing)
        if row < 0 or col < 0:
            sys.exit("Error: takeoff_pt is outside the DSM boundaries")
        self._ref_alti = self._np_dsm[row, col]
    
    def dsm_profile(self):
        """
        Makes a profile in the DSM
        
        Populates self._profile -> list of (e, n, z) coordinates
        Horizontal distance between each point = the DSM GSD
        
        NOTE: !WE SHOULD ADD THE CASES WHERE a_col == b_col AND a_row == b_row!
        """
        
        self._profile = []
        nb_rows = self._np_dsm.shape[1]
        
        # (col,row) system: origin at the top-left of the image, col towards the
        # right and row downwards
        # (x,y) system: origin at the bottom-left of the image, x towards the
        # right and y upwards
        
        # (AB): y = mx + p with A = profile start and B = profile end
        
        # Image coordinates
        a_col = (self._a_east  - self._top_left_e) / self._x_spacing
        a_row = (self._a_north - self._top_left_n) / self._y_spacing
        b_col = (self._b_east  - self._top_left_e) / self._x_spacing
        b_row = (self._b_north - self._top_left_n) / self._y_spacing
        
        # Change coordinate system: col, row -> x, y
        a_x = a_col
        a_y = nb_rows - a_row
        b_x = b_col
        b_y = nb_rows - b_row
        
        m = (b_y - a_y) / (b_x - a_x)
        p = a_y - m * a_x
        
        # Extract the profile
        # http://stackoverflow.com/a/7880726
        # !! Il faut parler à numpy avec x et y inversés !!
        alpha = atan(m)
        d = cos(alpha) # Distance on the abscissa for a distance of 1 on the
                       # profile
        
        if b_col < a_col:
            col_min = b_col
            col_max = a_col
            reverse = True
        else:
            col_min = a_col
            col_max = b_col
            reverse = False
        
        # Add a margin to each end, because as we estimate the drone orientation
        # by using the neighborhood, we need extra DSM information to estimate
        # the orientation of both endpoints (+1 by safety)
        self._dsm_profile_margin = ceil(self._footprint/2/self._x_spacing) + 1
        col_min -= ceil(self._dsm_profile_margin * d) # Projected along x
        col_max += ceil(self._dsm_profile_margin * d) # Same
        
        col_list = np.arange(col_min, col_max, d)
        
        row_list = []
        for col in col_list:
            row_list.append(nb_rows - (m * col + p)) # Change coord. system:
                                                     # x, y -> col, row
        row_list = np.array(row_list)
        
        if reverse:
            col_list = np.flip(col_list, 0)
            row_list = np.flip(row_list, 0)
        
        # Nearest neighbour
        z_list = self._np_dsm[row_list.astype(np.int), col_list.astype(np.int)]
        
        for coord in zip(col_list, row_list, z_list):
            e = coord[0] * self._x_spacing + self._top_left_e
            n = coord[1] * self._y_spacing + self._top_left_n
            self._profile.append({'e':e, 'n':n, 'z':coord[2]})
    
    def find_ori(self, index, locked=False):
        """
        Finds the gimbal pitch angle of the drone and its azimuth for the
        current position
        
        By position, the position "on the ground" is meant. Next need to apply
        the desired (fixed) height from the ground (vertical or not, depending
        on the pitch), from this position. This is done by apply_height().
        
        Input:
            index           [int] index of the current position in self._profile
            locked          [bool] wether this orientation can be moved or not
        
        Output:
            index           [int] same as the input
            pitch           [float] gimbal pitch for the current position (in
                                rad)
            drone_az        [bool] drone azimuth for the current position;
                                True  = same direction as the profile;
                                False = direction opposite to the profile
                                direction
            footp_i_start   [int] index of the beginning of the footprint
            footp_i_end     [int] index of the end of the footprint
            locked          [bool] same as the input
        """
        
        # To estimate the pitch, we need to evaluate the mean slope of the zone
        # captured by the image. To do so, we start at the index given in input
        # and we explore the DSM on both sides, index by index, until the length
        # covered on a plane of slope = the mean slope is reached. This way, the
        # DSM values taken into account correspond to the actual footprint that
        # the image will encompass.
        ########
        # TODO
        # Maybe it is a good idea to limit the pitch values, so that it is not
        # too far from nadir?
        ########
        inc = 1
        slopes = [] # Collect the slope between each encompassed DSM index
        while True:
            delta_z_left = self._profile[index - inc]['z'] - \
                self._profile[index - inc + 1]['z']
            delta_z_right = self._profile[index + inc - 1]['z'] - \
                self._profile[index + inc]['z']
            
            slopes.extend((atan(delta_z_left / self._x_spacing),
                atan(delta_z_right / self._x_spacing)))
            
            # Stop the loop when the length of the mean plane exceeds the
            # footprint
            l = (2 * inc * self._x_spacing) / cos(np.mean(slopes))
            if l > self._footprint:
                break
            
            inc += 1
        
        footp_i_start = index - inc
        footp_i_end = index + inc
        
        if not self._fixed_pitch:
            pitch = np.mean(slopes) + pi/2 # True whether topography goes up or
                                           # down
            drone_az = True if pitch <= pi/2 else False # If topography goes
                                                        # down, turn the drone
                                                        # over to take the photo
            pitch = pi - pitch if drone_az == False else pitch
            pitch *= -1 # Will always shoot down, never up
        else:
            # Pitch should always be negative, but by convention, a positive
            # pitch means that we want the drone to shoot backwards
            if self._fixed_pitch > 0:
                pitch = -radians(self._fixed_pitch)
                drone_az = False
            else:
                pitch = radians(self._fixed_pitch)
                drone_az = True
        
        if pitch > 0:
            print("Warning: pitch is positive (should always be negative)")
        
        return {
                'index':            index,
                'pitch':            pitch,
                'drone_az':         drone_az,
                'footp_i_start':    footp_i_start,
                'footp_i_end':      footp_i_end,
                'locked':           locked
            }
    
    def apply_height(self, index, pitch, drone_az, footp_i_start, footp_i_end,
            locked):
        """
        Applies the desired flight distance at the position found by find_ori(),
        taking the pitch into account
        
        Input:
            index           [int] index of the current position in self._profile
            pitch           [float] gimbal pitch for the current position (in
                                rad)
            drone_az        [bool] drone azimuth for the current position;
                                True  = same direction as the profile;
                                False = direction opposite to the profile
                                direction
            footp_i_start   [int] index of the beginning of the footprint
            footp_i_end     [int] index of the end of the footprint
            locked          [bool] same as the input
        
        Output:
            index           [int] same as the input
            e_grd           [float] (e, n, z) coordinates on the ground, before
            n_grd               applying the flight distance, pitch and drone_az
            z_grd               (in m)
            index_abv       [int] profile index corresponding to the position of
                                the drone above the ground, considering the
                                flight distance, pitch and drone_az
            e_abv           [float] (e, n, z) coordinates of the drone above the
            n_abv               ground, considering the flight distance, pitch
            z_abv               and drone_az (in m)
            pitch           [float] same as the input
            drone_az        [bool] same as the input
            footp_i_start   [int] same as the input
            footp_i_end     [int] same as the input
            locked          [bool] same as the input
        """
        
        e = self._profile[index]['e']
        n = self._profile[index]['n']
        
        delta_index = abs(self._h * cos(pitch)) # Distance between the current
                                                # position and the new position,
                                                # projected along the profile
                                                # axis
        
        # Difference in E and N between the two positions
        delta_e = abs(delta_index * sin(self._prof_az))
        delta_n = abs(delta_index * cos(self._prof_az))
        
        if drone_az: # The drone shoots in the same direction as the profile
            index_abv = index - delta_index / self._x_spacing
            if self._prof_az >= 0 and self._prof_az < pi/2:
                e_abv = e - delta_e
                n_abv = n - delta_n
            elif self._prof_az >= pi/2 and self._prof_az < pi:
                e_abv = e - delta_e
                n_abv = n + delta_n
            elif self._prof_az >= pi and self._prof_az < 3*pi/2:
                e_abv = e + delta_e
                n_abv = n + delta_n
            elif self._prof_az >= 3*pi/2 and self._prof_az < 2*pi:
                e_abv = e + delta_e
                n_abv = n - delta_n
        else: # The drone shoots to the direction opposite to the profile
            index_abv = index + delta_index / self._x_spacing
            if self._prof_az >= 0 and self._prof_az < pi/2:
                e_abv = e + delta_e
                n_abv = n + delta_n
            elif self._prof_az >= pi/2 and self._prof_az < pi:
                e_abv = e + delta_e
                n_abv = n - delta_n
            elif self._prof_az >= pi and self._prof_az < 3*pi/2:
                e_abv = e - delta_e
                n_abv = n - delta_n
            elif self._prof_az >= 3*pi/2 and self._prof_az < 2*pi:
                e_abv = e - delta_e
                n_abv = n + delta_n
        
        z = self._profile[index]['z']
        z_abv = z + abs(self._h * sin(pitch))
        
        return {
                'index':            index,
                'e_grd':            e,
                'n_grd':            n,
                'z_grd':            z,
                'index_abv':        index_abv,
                'e_abv':            e_abv,
                'n_abv':            n_abv,
                'z_abv':            z_abv,
                'pitch':            pitch,
                'drone_az':         drone_az,
                'footp_i_start':    footp_i_start,
                'footp_i_end':      footp_i_end,
                'locked':           locked
            }
    
    def add_ori(self, index, locked=False):
        """
        Adds an orientation along the profile, at the given index
        
        Input:
            index           [int] profile index where to add the orientation
            locked          [bool] wether the index of this orientation can be
                                shifted or not
        
        Output:
            ori_w_pitch     [dict] new orientation
        """
        
        ori = self.find_ori(index, locked=locked)
        ori_w_pitch = self.apply_height(
                ori['index'],
                ori['pitch'],
                ori['drone_az'],
                ori['footp_i_start'],
                ori['footp_i_end'],
                ori['locked']
            )
        return ori_w_pitch
    
    def estim_overlp(self, index1, index2):
        """
        Estimates the overlap ratio between two images
        
        Gathers all the values from the profile, that are included in the union
        of the fooprints of the images. Then, calculates the linear regression
        of this dataset.
        
        Input:
            index1          [int] profile index of the first orientation
            index2          [int] profile index of the second orientation
        
        Output:
            overlap ratio   [float] overlap ratio between the two orientations
        """
        
        o1 = self._drone_ori[index1]
        o2 = self._drone_ori[index2]
        
        profile_elev = [v['z'] for v in self._profile]
        
        x = np.arange(o1['footp_i_start'], o2['footp_i_end']+1).reshape((-1, 1))
        z = profile_elev[o1['footp_i_start']:o2['footp_i_end']+1]
        model = LinearRegression()
        model.fit(x, z)
        a = model.coef_[0]
        b = model.intercept_
        
        x = np.array([o1['footp_i_start'],
             o2['footp_i_start'],
             o1['footp_i_end'],
             o2['footp_i_end']]).reshape((-1, 1))
        z = model.predict(x)
        
        # To show the linear regressions in draw_orientations()
        self._ovlp_linreg_x[index2] = list(x.reshape((1, -1))[0])
        self._ovlp_linreg_z[index2] = list(z)
        self._ovlp_linreg_stats[index2] = [round(a, 2),
            round(b, 1), round(model.score(x, z), 2)]
        
        # Overlap between the two images
        L = sqrt((x[3]-x[0])**2 + (z[3]-z[0])**2) # Union of the footprints
        l = sqrt((x[2]-x[1])**2 + (z[2]-z[1])**2) # Overlapping part
        
        return l/L
    
    def estim_b_on_h(self, index1, index2):
        """
        Estimates the B/H ratio between two images
        """
        
        
    
    def insert_ori(self, index1, index2):
        """
        Inserts an orientation between two orientations
        
        Input:
            index1          [int] profile index of the first orientation
            index2          [int] profile index of the second orientation
        
        Output:
            new_ori_w_pitch [dict] new orientation
        """
        
        o1 = self._drone_ori[index1]
        o2 = self._drone_ori[index2]
        new_ori_index = int(o1['index'] + (o2['index'] - o1['index']) / 2)
        return self.add_ori(new_ori_index)
    
    def drone_orientations(self):
        """
        Estimates the drone orientation at each shot, from the profile extracted
        from the DSM
        
        Note: we need to take into account the margin (dsm_profile_margin), to
              localize where the profile starts and where it ends, in the
              profile list
        """
        
        self._drone_ori = {}
        self._ovlp_linreg_x = {}
        self._ovlp_linreg_z = {}
        self._ovlp_linreg_stats = {}
        self._final_overlap = {}
        
        # Initialization with the first and last points of the profile
        # Start of profile
        index = self._dsm_profile_margin
        ori_start = self.add_ori(index, locked=True)
        self._drone_ori[ori_start['index']] = ori_start
        
        # End of profile
        index = len(self._profile)-self._dsm_profile_margin
        ori_end = self.add_ori(index, locked=True)
        self._drone_ori[ori_end['index']] = ori_end
        
        drone_ori_by_keys = sorted(list(self._drone_ori.keys()))
        
        # Add orientations until the required overlap and B/H ratios are
        # satisfied
        while True:
            i = 0
            new_drone_ori = {}
            while i < len(self._drone_ori)-1:
                o1 = self._drone_ori[drone_ori_by_keys[i]]
                o2 = self._drone_ori[drone_ori_by_keys[i+1]]
                index1 = o1['index']
                index2 = o2['index']
                
                if o1['footp_i_end'] < o2['footp_i_start']: # If o1 and o2
                                                            # footprints do not
                                                            # intersect
                    # Add an orientation between them
                    new_ori = self.insert_ori(index1, index2)
                    new_drone_ori[new_ori['index']] = new_ori
                else: # If o1 and o2 do intersect, make extra tests
                    # Check the overlap ratio
                    overlap = self.estim_overlp(index1, index2)
                    self._final_overlap[index2] = round(overlap, 2)
                    if overlap < self._ovlp: # If not enough overlap
                        # If not locked, reduce the index to increase overlap,
                        # then run the loop again, without adding any other
                        # orientations
                        if not o2['locked']:
                            # Remove o2
                            self._drone_ori.pop(index2)
                            self._ovlp_linreg_x.pop(index2)
                            self._ovlp_linreg_z.pop(index2)
                            self._ovlp_linreg_stats.pop(index2)
                            self._final_overlap.pop(index2)
                            
                            drone_ori_by_keys = sorted(list( # Update the list
                                self._drone_ori.keys()))
                            new_o2 = self.add_ori(index2-1) # VERY COSTLY! Maybe
                                                            # adding one between
                                                            # o1 and o2 would be
                                                            # better
                            new_drone_ori[new_o2['index']] = new_o2
                            break
                        else: # If locked
                            # Add an orientation between them
                            new_ori = self.insert_ori(index1, index2)
                            new_drone_ori[new_ori['index']] = new_ori
                    
                    # Check the B/H ratio
                    # To be done, if needed
                
                i += 1
            
            # Check for duplicated keys
            new_drone_ori_by_keys = list(new_drone_ori.keys())
            for k in new_drone_ori_by_keys:
                if k in drone_ori_by_keys:
                    print(
                        "Warning: duplicated key {} in new_drone_ori".format(k))
            
            # If no new orientation were added in this loop, break
            if not new_drone_ori:
                break
            
            self._drone_ori.update(new_drone_ori)
            drone_ori_by_keys = sorted(list(self._drone_ori.keys()))
        
    def draw_orientations(self, disp_drone_pos=True, disp_footp=False,
            disp_fov=True, disp_linereg=False, print_pitch=False,
            print_ovlp=False, print_linereg=False):
        """
        Draws the DSM profile with the drone orientations
        
        Input:
            disp_drone_pos  [bool] wether or not to display the drone positions
                                (default is True)
            disp_footp      [bool] wether or not to display the image footprint
                                (default is False)
            disp_fov        [bool] wether or not to display the camera field of
                                view (default is True)
            disp_linereg    [bool] wether or not to display the linear
                                regressions used to estimate the overlap ratio
                                (default is False)
            print_pitch     [bool] wether or not to print the pitch value of
                                each orientation, in the terminal (default is
                                False)
            print_ovlp      [bool] wether or not to print the overlap ratio of
                                each pair of orientations, in the terminal
                                (default is False)
            print_linereg   [bool] wether or not to print the linear regression
                                information for each pair of orientation, in the
                                terminal (default is False)
        
        Output:
            an image is written in the working directory
        """
        
        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.figsize'] = (16, 4)
        
        GRID_LINESTYLE = (0, (20, 20))
        GRID_LINEWIDTH = 0.2
        
        fig, ax = plt.subplots()
        
        # DSM profile
        prof_z = [v['z'] for v in self._profile]
        prof_x = np.arange(0, len(prof_z)) * self._x_spacing
        ax.plot(prof_x, prof_z, linewidth=0.3)
        
        # Extract info from the drone orientations
        drone_ori_by_keys = sorted(list(self._drone_ori.keys()))
        drone_x_abv = []
        drone_z_abv = []
        drone_color = []
        pitch_dict = {}
        footp_lines = []
        fov_lines = []
        for i in drone_ori_by_keys:
            o = self._drone_ori[i]
            
            x_abv = o['index_abv']
            z_abv = o['z_abv']
            look_dir = o['drone_az']
            pitch = o['pitch']
            
            drone_x_abv.append(x_abv)
            drone_z_abv.append(z_abv)
            
            pitch_dict[i] = degrees(pitch)
            
            if look_dir:
                drone_color.append('r')
            else:
                drone_color.append('g')
            
            # Footprint
            footp_i_start = o['footp_i_start']
            footp_i_end = o['footp_i_end']
            footp_z_start = prof_z[footp_i_start]
            footp_z_end = prof_z[footp_i_end]
            footp_lines.extend([
                    [(x_abv, z_abv), (footp_i_start, footp_z_start)],
                    [(x_abv, z_abv), (footp_i_end, footp_z_end)]
                ])
            
            # Field of view (very similar to footprint, probably more realistic)
            SIGHT_LEN = 15
            sight1 = pitch - self._fov_lon / 2
            sight2 = pitch + self._fov_lon / 2
            if not look_dir:
                sight1_x = x_abv - cos(sight1) * SIGHT_LEN
                sight2_x = x_abv - cos(sight2) * SIGHT_LEN
            else:
                sight1_x = x_abv + cos(sight1) * SIGHT_LEN
                sight2_x = x_abv + cos(sight2) * SIGHT_LEN
            sight1_z = z_abv + sin(sight1) * SIGHT_LEN
            sight2_z = z_abv + sin(sight2) * SIGHT_LEN
            fov_lines.extend([
                    [(x_abv, z_abv), (sight1_x, sight1_z)],
                    [(x_abv, z_abv), (sight2_x, sight2_z)]
                ])
        
        # Drone position
        if disp_drone_pos:
            drone_pos = ax.scatter(drone_x_abv, drone_z_abv, c=drone_color, s=8)
        
        # Display footprint
        if disp_footp:
            lc = mc.LineCollection(footp_lines, linewidths=0.1)
            ax.add_collection(lc)
        
        # Display field of view
        if disp_fov:
            lc = mc.LineCollection(fov_lines, linewidths=0.1, color='k')
            ax.add_collection(lc)
        
        # Display the linear regressions used to estimate the overlap ratio
        if disp_linereg:
            ovlp_linreg_x_by_keys = sorted(list(self._ovlp_linreg_x.keys()))
            ovlp_linreg_x = []
            for i in ovlp_linreg_x_by_keys:
                ovlp_linreg_x.append(self._ovlp_linreg_x[i])
            ovlp_linreg_z_by_keys = sorted(list(self._ovlp_linreg_z.keys()))
            ovlp_linreg_z = []
            for i in ovlp_linreg_z_by_keys:
                ovlp_linreg_z.append(self._ovlp_linreg_z[i])
            linear_regs = list(zip(ovlp_linreg_x, ovlp_linreg_z))
            lines = []
            for elem in linear_regs:
                zipped = [list(t) for t in zip(elem[0], elem[1])]
                lines.append(zipped)
            lc = mc.LineCollection(lines, linewidths=0.5, color='r')
            ax.add_collection(lc)
            x = [i[0] for j in lines for i in j]
            y = [i[1] for j in lines for i in j]
            ax.plot(x, y, '.', markersize=0.4, color='k')
        
        # Print the pitch for each orientation, in the terminal
        if print_pitch:
            print('Pitch:\nindex: pitch')
            pprint.pprint(pitch_dict)
        
        # Print the overlap ratio of each pair, in the terminal
        if print_ovlp:
            print('Overlap ratios:\nindex: overlap')
            pprint.pprint(self._final_overlap)
        
        # Print the linear regression information for each pair, in the terminal
        if print_linereg:
            print('Linear regressions:\nindex: [a, b, r2_score]')
            pprint.pprint(self._ovlp_linreg_stats)
        
        # Add the location of points A and B
        locs = list(ax.get_xticks())
        locs_for_lab = []
        for l in locs:
            locs_for_lab.append(round(l))
            
        ax.set_xticks(ax.get_xticks())  # just get and reset to prevent annoying bug
        labels = ax.set_xticklabels(locs_for_lab)
        x_a = self._dsm_profile_margin * self._x_spacing
        x_b = (len(prof_z) - self._dsm_profile_margin) * self._x_spacing
        locs += [x_a, x_b]
        ax.tick_params(axis='x', direction='inout')
        ax.set_xticks(locs)
        
        # Grid
        plt.grid(linestyle=GRID_LINESTYLE, linewidth=GRID_LINEWIDTH)
        
        # Title
        pitch_txt = 'Pitch: '
        if self._fixed_pitch:
            pitch_txt += '{}° (fixed)'.format(self._fixed_pitch)
        else:
            pitch_txt += '<free>'
        title = ('{}\nFlight dist.: {} m, Overlap: {:.0f}%, {}, '
            'FoV: {:.0f}°, GSD: {:.1e} m'.format(
                self._name,
                self._h,
                100*self._ovlp,
                pitch_txt,
                degrees(self._fov_lon),
                self._gsd
            ))
        title += '\n{} images'.format(len(self._drone_ori))
        plt.title(title)
        
        # Axis titles
        plt.xlabel('Distance along the profile (m)')
        plt.ylabel('Z (m)')
        
        # Legend
        r_marker = mpatches.Patch(color='r', label='forwards')
        g_marker = mpatches.Patch(color='g', label='backwards')
        ax.legend(handles=[r_marker, g_marker])
        
        plt.gca().set_aspect('equal')
        
        xmin = min(prof_x)
        xmax = max(prof_x)
        xmin -= 0.05 * (xmax - xmin)
        xmax += 0.05 * (xmax - xmin)
        ax.set(xlim=(xmin, xmax))
        
        zmin = min(min(prof_z), min(drone_z_abv))
        zmax = max(max(prof_z), max(drone_z_abv))
        zmin -= 0.2 * (zmax - zmin)
        zmax += 0.2 * (zmax - zmin)
        ax.set(ylim=(zmin, zmax))
        
        ax.text(x_a, zmin+0.02*(zmax-zmin), 'A', ha='center')
        ax.text(x_b, zmin+0.02*(zmax-zmin), 'B', ha='center')
        
        plt.tight_layout()
        
        fig.savefig('{}_orientations.svg'.format(self._name),
            bbox_inches='tight')
    
    def draw_map(self, shaded_dsm=None, make_subset=False, disp_dsm_prof=True,
            disp_drone_grd=True, disp_drone_pos=True, disp_takeoff_pt=True):
        """
        Draws the path with all orientations on a map
        
        Input:
            shaded_dsm      [string] path to the shaded DSM .tif file;
                                it must have the same projection and the same
                                extent as the DSM (default is None, as it is
                                optional)
            make_subset     [bool] wether or not to adapt the range of the
                                shaded DSM to the plotted path (default is
                                False)
            disp_dsm_prof   [bool] wether or not to display the DSM profile
                                (default is True)
            disp_drone_grd  [bool] wether or not to display the position
                                position "on the ground", i.e. more or less the
                                projection of the optical center on the ground
                                (default is True)
            disp_drone_pos  [bool] wether or not to display the drone positions
                                (default is True)
            disp_takeoff_pt [bool] wether or not to display the drone takeoff
                                position, whenever it exists (default is True)
        
        Output:
            an image is written in the working directory
        """
        
        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.figsize'] = (8, 8)
        
        fig, ax = plt.subplots()
        
        handles = []
        labels = []
        
        # Extract info from the drone orientations
        drone_ori_by_keys = sorted(list(self._drone_ori.keys()))
        drone_e_grd = []
        drone_n_grd = []
        drone_e_abv = []
        drone_n_abv = []
        for i in drone_ori_by_keys:
            o = self._drone_ori[i]
            
            e_grd = o['e_grd']
            n_grd = o['n_grd']
            e_abv = o['e_abv']
            n_abv = o['n_abv']
            
            drone_e_grd.append(e_grd)
            drone_n_grd.append(n_grd)
            drone_e_abv.append(e_abv)
            drone_n_abv.append(n_abv)
        
        colors = []
        for i, v in enumerate(drone_e_grd):
            # Alternate between 3 colors
            if i%3 is 0:
                c = 'r'
            elif i%3 is 1:
                c = 'g'
            elif i%3 is 2:
                c = 'b'
            colors.append(c)
        
        # DSM profile
        if disp_dsm_prof:
            e = [v['e'] for v in self._profile]
            n = [v['n'] for v in self._profile]
            ax.plot(e, n, alpha=0.3, zorder=1)
            
            a_e = self._a_east
            b_e = self._b_east
            a_n = self._a_north
            b_n = self._b_north
            c = 'yellow' if shaded_dsm is not None else 'k'
            ax.plot([a_e, b_e], [a_n, b_n], 'x', c=c, zorder=2)
            ax.annotate('A', xy=[a_e, a_n])
            ax.annotate('B', xy=[b_e, b_n])
        
        # Drone takeoff position
        if disp_takeoff_pt and self._takeoff_pt:
            takeoff = ax.scatter(self._takeoff_pt[0], self._takeoff_pt[1],
                s=150, c='yellow', marker='*', edgecolors='k', linewidths=0.2)
            handles.append(takeoff)
            labels.append('takeoff position')
        
        # Drone position "on the ground". It is more or less the projection of
        # the optical center on the ground
        if disp_drone_grd:
            drone_grd = ax.scatter(drone_e_grd, drone_n_grd, s=2, c=colors,
                linewidths=0.1, edgecolors='k', zorder=3)
            handles.append(drone_grd)
            labels.append('target')
        
        # Drone position, in planimetry
        if disp_drone_pos:
            drone_pos = ax.scatter(drone_e_abv, drone_n_abv, s=1, c=colors,
                linewidths=0.1, edgecolors='k', zorder=4)
            handles.append(drone_pos)
            labels.append('drone')
        
        # Shaded DSM
        if shaded_dsm is not None:
            dsm_img = mpimg.imread(shaded_dsm)
            
            if make_subset:
                e_min, e_max = ax.get_xlim()
                n_min, n_max = ax.get_ylim()
                
                x_min = int(floor((e_min - self._top_left_e) / self._x_spacing))
                x_max = int(ceil((e_max - self._top_left_e) / self._x_spacing))
                y_min = int(floor((n_min - self._top_left_n) / self._y_spacing))
                y_max = int(ceil((n_max - self._top_left_n) / self._y_spacing))
                
                subset_x = range(x_min, x_max+1)
                subset_y = range(y_max, y_min+1)
                
                dsm_img = dsm_img[np.ix_(subset_y, subset_x)]
            else:
                e_min = self._top_left_e
                e_max = self._top_left_e + self._x_spacing * dsm_img.shape[1]
                n_min = self._top_left_n + self._y_spacing * dsm_img.shape[0]
                n_max = self._top_left_n
            
            ax.imshow(
                dsm_img, 'gray', extent=(e_min, e_max, n_min, n_max),
                zorder=0, alpha=0.7
            )
        
        # Title
        pitch_txt = 'Pitch: '
        if self._fixed_pitch:
            pitch_txt += '{}° (fixed)'.format(self._fixed_pitch)
        else:
            pitch_txt += '<free>'
        title = ('{}\nFlight dist.: {} m, Overlap: {:.0f}%, {}, '
            'FoV: {:.0f}°, GSD: {:.1e} m'.format(
                self._name,
                self._h,
                100*self._ovlp,
                pitch_txt,
                degrees(self._fov_lon),
                self._gsd
            ))
        title += '\n{} images'.format(len(self._drone_ori))
        plt.title(title)
        
        # Axis titles
        plt.xlabel('Easting (m)')
        plt.ylabel('Northing (m)')
        
        plt.yticks(rotation=90)
        
        # Legend
        if handles:
            ax.legend(handles, labels)
        
        plt.gca().set_aspect('equal')
        plt.tight_layout()
        
        fig.savefig('{}_map.svg'.format(self._name), bbox_inches='tight')
    
    def export_ori(self):
        """
        Exports all orientation information
        
        The export consists in a list of dict that contains, for each
        orientation:
            - path_name:            name of the drone path
            - path_az:              azimuth of the path (in deg)
            - drone_e, _n, _z:      3D coordinates of the drone (in m)
            - drone_pitch:          drone gimbal pitch angle (in deg)
            - drone_fov_lat, _lon:  camera field of view in the longitudinal and
                                        lateral directions (in deg)
            - drone_az:             drone azimuth;
                                        True  = same direction as the profile;
                                        False = direction opposite to the
                                        profile direction
        """
        
        export = []
        for ori in self._drone_ori:
            o = self._drone_ori[ori]
            export.append({
                    'path_name':        self._name,
                    'path_az':          degrees(self._prof_az),
                    'drone_e':          o['e_abv'],
                    'drone_n':          o['n_abv'],
                    'drone_z':          o['z_abv'] - self._ref_alti,
                    'drone_pitch':      degrees(o['pitch']),
                    'drone_fov_lat':    degrees(self._fov_lat),
                    'drone_fov_lon':    degrees(self._fov_lon),
                    'drone_az':         o['drone_az'],
                    'index_abv':        o['index_abv']
                })
        export = sorted(export, key=lambda e: e['index_abv'])
        
        for o in export:
            o.pop('index_abv', None) # Remove index_abv, once export is sorted
        
        return export
