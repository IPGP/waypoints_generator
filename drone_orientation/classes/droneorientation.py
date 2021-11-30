#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
from math import tan, radians, degrees, atan, cos, ceil, sqrt, floor, pi, sin
import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib
matplotlib.use('svg')
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.patches as mpatches
import pprint

class DroneOri(object):
    """
    Define the successive positions (orientations) of the drone along the path,
    to make acquisitions with a defined overlap ratio from one image to the
    next
    
    Variables:
        name                [string] name of the drone path
        np_dsm              [np array] the DSM
        tfw                 [string] path to the DSM .tfw file
        a_east, a_north     [floats] position of 1st pt of the profile in the
                                DSM (in degrees or meters, depending on the
                                projection)
        b_east, b_north     [floats] position of last pt of the profile in the
                                DSM in degrees or meters, depending on the
                                projection)
        h                   [float] distance between the drone and the ground
                                (in meters)
        fov                 [float] camera field of view (in degrees)
        ovlp                [float] overlap ratio between two consecutive images
                                (shoud be between 0.5 and 0.95)
        footprint           [float] image footprint (in meters)
        profile             [list] profile between a and b in the DSM (e, n, z)
        prof_az             [float] azimuth of the profile (in radians)
        x_spacing, y        [float] DSM GSD
        top_left_e, n       [float] coordinates of the top-left corner of the
                                DSM top-left px
        dsm_profile_margin  [int] margin to cover both endpoints of the profile,
                                = ceil(footprint/2/x_spacing) + 1 (w/o units)
                                (i.e. nb of extra values that were added to each
                                end of self._profile, +1 by safety)
        drone_ori           [list] drone orientation (e, n, z, pitch, az)
    """
    
    def __init__(self, name, np_dsm, tfw, a_east, a_north, b_east, b_north, h,
            fov, ovlp, footprint=None, profile=None, prof_az=None,
            x_spacing=None, y_spacing=None, top_left_e=None, top_left_n=None,
            dsm_profile_margin=None, drone_ori=None):
        self._name = name
        self._np_dsm = np_dsm
        self._tfw = tfw
        self._a_east = a_east
        self._a_north = a_north
        self._b_east = b_east
        self._b_north = b_north
        self._h = h
        self._fov = fov
        self._ovlp = ovlp
        self._footprint = footprint
        self._profile = profile
        self._prof_az = prof_az
        self._x_spacing = x_spacing
        self._y_spacing = y_spacing
        self._top_left_e = top_left_e
        self._top_left_n = top_left_n
        self._dsm_profile_margin = dsm_profile_margin
        self._drone_ori = drone_ori
        
        self.compute_prof_az()
        self.compute_footprint()
        self.read_tfw()
        
        # For tests
        self._ovlp_linreg_x = {}
        self._ovlp_linreg_z = {}
        self._ovlp_linreg_stats = {}
        self._final_overlap = {}
    
    @property
    def name(self):
        return self._name
    
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
    def h(self):
        return self._h
    
    @property
    def fov(self):
        return self._fov
    
    @property
    def ovlp(self):
        return self._ovlp
    
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
    
    def compute_prof_az(self):
        """
        Compute the azimuth of the profile
        
        0 is the North; positive clockwise
        """
        
        delta_e = self._b_east - self._a_east
        delta_n = self._b_north - self._a_north
        
        if delta_e > 0:
            if delta_n < 0:
                self._prof_az = abs(atan(delta_e / delta_n))
            elif delta_n > 0:
                self._prof_az = abs(atan(delta_n / delta_e)) + pi/2
            else:
                self._prof_az = pi/2
        elif delta_e < 0:
            if delta_n > 0:
                self._prof_az = abs(atan(delta_e / delta_n)) + pi
            elif delta_n < 0:
                self._prof_az = abs(atan(delta_n / delta_e)) + 3*pi/2
            else:
                self._prof_az = 3*pi/2
        else:
            if delta_n > 0:
                self._prof_az = 0
            elif delta_n < 0:
                self._prof_az = pi
            else:
                sys.exit("Error: delta_e = 0 and delta_n = 0")
    
    def compute_footprint(self):
        """
        Compute the footprint of an image, based on h and fov
        """
        
        self._footprint = 2 * self._h * tan(radians(self._fov) / 2)
    
    def read_tfw(self):
        """
        Read georeferencing information from the DSM .tfw file
        
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
    
    def dsm_profile(self):
        """
        Make a profile in the DSM
        
        Populate self._profile -> list of (e, n, z) coordinates
        Horizontal distance between each point = the DSM GSD
        
        NOTE: !WE SHOULD ADD THE CASES WHERE a_col == b_col AND a_row == b_row!
        """
        
        self._profile = []
        nb_rows = self._np_dsm.shape[1]
        
        # (col,row) system: origin at the top-left of the image, col towards the
        # right and row downwards
        # (x,y) system: origin at the bottom-left of the image, x towards the
        # right and y upwards
        
        # (AB): y = mx + p with A = profile start and B = end of profile
        
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
        col_min -= ceil(self._dsm_profile_margin * d) # Projeté sur absc
        col_max += ceil(self._dsm_profile_margin * d) # Idem
        
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
    
    def find_ori(self, index):
        """
        Find the gimbal pitch angle of the drone and its azimuth for the current
        position.
        By position, I mean the position "on the ground". Next we will need to
        apply the desired (fixed) height from the ground (vertical or not,
        depending on the pitch), from this position.
        
        Input:
            index           [int] index of the current position in self._profile
        
        Output:
            pitch           [float] gimbal pitch for the current position (in
                                radians)
            drone_az        [bool] drone azimuth for the current position
                                True  = same direction as the profile
                                False = direction opposite to the profile
                                direction
            index           [int] same as the input
            footp_i_start   [int] index of the beginning of the footprint
            footp_i_end     [int] index of the end of the footprint
        """
        
        # To estimate the pitch, we need to evaluate the mean slope of the zone
        # captured by the image. To do so, we start at the index given in input
        # and we explore the DSM on both sides, step by step, until the length
        # covered on a plane of slope = the mean slope is reached. This way, the
        # DSM values taken into account correspond to the actual footprint that
        # the image will encompass.
        ###
        # TODO?
        # Maybe it is a good idea to limit the pitch values, so that it is not
        # too far from nadir?
        ###
        inc = 1
        slopes = [] # Collect the slope between each encompassed DSM indexes
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
        
        pitch = np.mean(slopes) + pi/2 # True whether topography goes up or down
        drone_az = True if pitch <= pi/2 else False # If topography goes down,
                                                    # turn the drone over to
                                                    # take the photo
        pitch = pi - pitch if drone_az == False else pitch
        pitch *= -1 # Will always shoot down, never up
        if pitch > 0:
            print("Warning: pitch is positive (should always be negative)")
        
        footp_i_start = index - inc
        footp_i_end = index + inc
        
        return {'pitch':            pitch,
                'drone_az':         drone_az,
                'index':            index,
                'footp_i_start':    footp_i_start,
                'footp_i_end':      footp_i_end}
    
    def apply_height(self, index, pitch, drone_az):
        """
        Apply the desired height to the current position, taking the pitch found
        by find_ori() into account
        
        Input:
            index       [int] index of the current position in self._profile
            pitch       [float] gimbal pitch for the current position (in
                            radians)
            drone_az    [bool] drone azimuth for the current position
                            True  = same direction as the profile
                            False = direction opposite to the profile direction
        
        Output:
            new_e       [float] (e, n, z) coordinates of the drone, considering
            new_n           the pitch and drone_az
            new_z       
            pitch       [float] same as the input
            drone_az    [bool] same as the input
        """
        
        e = self._profile[index]['e']
        n = self._profile[index]['n']
        
        delta_index = self._h * cos(pitch) # Distance between the current
                                           # position and the new position,
                                           # projected along the profile axis
        
        # Difference in E and N between the two positions
        delta_e = abs(delta_index * sin(self._prof_az))
        delta_n = abs(delta_index * cos(self._prof_az))
        
        if drone_az: # The drone shoots in the same direction as the profile
            if self._prof_az >= 0 and self._prof_az < pi/2:
                new_e = e + delta_e
                new_n = n + delta_n
            elif self._prof_az >= pi/2 and self._prof_az < pi:
                new_e = e + delta_e
                new_n = n - delta_n
            elif self._prof_az >= pi and self._prof_az < 3*pi/2:
                new_e = e - delta_e
                new_n = n - delta_n
            elif self._prof_az >= 3*pi/2 and self._prof_az < 2*pi:
                new_e = e - delta_e
                new_n = n + delta_n
        else: # The drone shoots to the direction opposite to the profile
            if self._prof_az >= 0 and self._prof_az < pi/2:
                new_e = e - delta_e
                new_n = n - delta_n
            elif self._prof_az >= pi/2 and self._prof_az < pi:
                new_e = e - delta_e
                new_n = n + delta_n
            elif self._prof_az >= pi and self._prof_az < 3*pi/2:
                new_e = e + delta_e
                new_n = n + delta_n
            elif self._prof_az >= 3*pi/2 and self._prof_az < 2*pi:
                new_e = e + delta_e
                new_n = n - delta_n
        
        new_z = self._profile[index]['z'] + abs(self._h * sin(pitch))
        
        return {'new_e':    new_e,
                'new_n':    new_n,
                'new_z':    new_z,
                'pitch':    pitch,
                'drone_az': drone_az}
    
    def add_ori(self, index, locked=False):
        """
        Add an orientation along the profile, at the given index
        
        Input:
            index           [int] index of the orientation to add
            locked          [bool] wether this orientation can be moved or not
        
        Output:
            ori_w_pitch     [dict] new orientation
        """
        
        ori = self.find_ori(index)
        ori_w_pitch = self.apply_height(
                ori['index'],
                ori['pitch'],
                ori['drone_az']
            )
        ori_w_pitch.update({
                'index':            ori['index'],
                'footp_i_start':    ori['footp_i_start'],
                'footp_i_end':      ori['footp_i_end'],
                'locked':           locked
            })
        return ori_w_pitch
    
    def estim_overlp(self, index1, index2):
        """
        Estimate the overlap ratio between two images
        Gather all values from the profile, that are included in the union of
        the fooprints of the images. Then, calculate the linear regression of
        this dataset.
        
        Input:
            index1          [int] index of the first orientation
            index2          [int] index if the second orientation
        
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
        
        # To show linear regressions in draw_orientations
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
        Estimate the B/H ratio between two images
        """
        
        
    
    def insert_ori(self, index1, index2):
        """
        Insert an orientation between two orientations
        
        Input:
            index1          [int] index of the first orientation
            index2          [int] index if the second orientation
        
        Output:
            new_ori_w_pitch [dict] new orientation
        """
        
        o1 = self._drone_ori[index1]
        o2 = self._drone_ori[index2]
        
        new_ori_index = int(o1['index'] + (o2['index'] - o1['index']) / 2)
        return self.add_ori(new_ori_index)
    
    def drone_orientations(self):
        """
        Estimate the drone orientation at each shot, from the profile extracted
        from the DSM
        
        Note: we need to take into account the margin (dsm_profile_margin), to
              localize where the profile starts and where it ends, in the list
              self._profile
        """
        
        self._drone_ori = {}
        
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
                            self._drone_ori.pop(index2) # Remove o2
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
            
            # If no new orientation were added, break the loop
            if not new_drone_ori:
                break
            
            self._drone_ori.update(new_drone_ori)
            drone_ori_by_keys = sorted(list(self._drone_ori.keys()))
        
    def draw_orientations(self, disp_dsm_prof=True, disp_drone_pos=True,
            disp_footp=False, disp_fov=True, disp_linereg=False):
        """
        Draw the DSM profile with the drone orientations
        """
        
        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.figsize'] = (16, 4)
        
        fig, ax = plt.subplots()
        
        plt.title(
            '{}\nFlight dist.: {} units, Field of view: {}°, Overlap: {}'.format(
                self._name,
                self._h,
                self._fov,
                self._ovlp
            ))
        
        # DSM profile
        if disp_dsm_prof:
            prof_z = [v['z'] for v in self._profile]
            prof_x = np.arange(0, len(prof_z))
            ax.plot(prof_x, prof_z, linewidth=0.3)
        
        # Extract info from drone orientations
        drone_ori_by_keys = sorted(list(self._drone_ori.keys()))
        drone_x = []
        drone_z = []
        drone_color = []
        pitch = []
        lines = []
        lines2 = []
        for i in drone_ori_by_keys:
            o = self._drone_ori[i]
            
            x = o['index']
            z = o['new_z']
            look_dir = o['drone_az']
            p = o['pitch']
            
            drone_x.append(x)
            drone_z.append(z)
            
            if look_dir:
                drone_color.append('r')
            else:
                drone_color.append('g')
            
            # Footprint
            footp_i_start = o['footp_i_start']
            footp_i_end = o['footp_i_end']
            footp_z_start = prof_z[footp_i_start]
            footp_z_end = prof_z[footp_i_end]
            lines.extend([[(x, z), (footp_i_start, footp_z_start)],
                [(x, z), (footp_i_end, footp_z_end)]])
            
            # Field of view (very similar to footprint, but more realistic)
            SIGHT_LEN = 30
            sight1 = p - radians(self._fov) / 2
            sight2 = p + radians(self._fov) / 2
            if not look_dir:
                sight1_x = x - cos(sight1) * SIGHT_LEN
                sight2_x = x - cos(sight2) * SIGHT_LEN
            else:
                sight1_x = x + cos(sight1) * SIGHT_LEN
                sight2_x = x + cos(sight2) * SIGHT_LEN
            sight1_z = z + sin(sight1) * SIGHT_LEN
            sight2_z = z + sin(sight2) * SIGHT_LEN
            lines2.extend([[(x, z), (sight1_x, sight1_z)],
                [(x, z), (sight2_x, sight2_z)]])
        
        # Drone position
        if disp_drone_pos:
            drone_pos = ax.scatter(drone_x, drone_z, c=drone_color, s=8)
        
        # Display footprint
        if disp_footp:
            lc = mc.LineCollection(lines, linewidths=0.1)
            ax.add_collection(lc)
        
        # Display field of view
        if disp_fov:
            lc = mc.LineCollection(lines2, linewidths=0.1, color='k')
            ax.add_collection(lc)
        
        # Linear regressions used to estimate the overlap ratio
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
            print('Overlap ratios:')
            pprint.pprint(self._final_overlap)
            print('Linear regressions:')
            pprint.pprint(self._ovlp_linreg_stats)
        
        # Legend
        r_marker = mpatches.Circle([], radius=5, color='r', label='forwards')
        g_marker = mpatches.Circle([], radius=5, color='g', label='backwards')
        ax.legend(handles=[r_marker, g_marker])
        
        plt.gca().set_aspect('equal')
        zmin = min(min(prof_z), min(drone_z))
        zmax = max(max(prof_z), max(drone_z))
        zmin -= 0.2 * (zmax - zmin)
        zmax += 0.2 * (zmax - zmin)
        ax.set(ylim=(zmin, zmax))
        
        plt.tight_layout()
        
        fig.savefig('{}_orientations.svg'.format(self._name))
