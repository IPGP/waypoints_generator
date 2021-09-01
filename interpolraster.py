#!/usr/bin/python
# -*- coding:utf-8 -*-

from scipy.interpolate import RectBivariateSpline

class Interpolate(object):
    """
    Interpolation in a raster, using scipy.interpolate.RectBivariateSpline
    
    Variables:
        np_raster   [np array] the raster in which to interpolate
        kx, ky      [ints] degrees of the bivariate spline. Default is 3.
        interpol    [scipy obj] the interpolation function
        z           interpolation result
    """
    
    def __init__(self, np_raster, kx=3, ky=3, interpol=None, z=None):
        self._np_raster = np_raster
        self._kx = kx
        self._ky = ky
        self._interpol = interpol
        self.init_interp_func()
    
    @property
    def np_raster(self):
        return self._np_raster
    
    @property
    def kx(self):
        return self._kx
    
    @property
    def ky(self):
        return self._ky
    
    @property
    def interpol(self):
        if not self._interpol:
            self.init_interp_func()
        return self._interpol
    
    def init_interp_func(self):
        """
        Instantiation of the interpolation function
        """
        
        self._interpol = RectBivariateSpline(
                range(self._np_raster.shape[0]),
                range(self._np_raster.shape[1]),
                self._np_raster, kx=self._kx, ky=self._ky
            )
    
    def interpolate(self, x, y):
        """
        Interpolation at (x, y)
        
        Input:
            x, y        [floats] coordinates to interpolate
        """
        
        if self._interpol is not None:
            return self._interpol.ev(x, y)
