import numpy as np

from . import resistance

class ShipDimensions():
    ''' 
    Class that stores main dimension fo a ship and calculates derived vairables 
    based on the main dimensions. Used as input to other equations that 
    calculate empirical values for model coefficients
    '''

    def __init__(self, L, B, T, volume, S=None):
        self.L = L
        self.B = B
        self.T = T
        self.volume = volume
        self.x_G = 0.0 # Center of gravity
        self.x_B = 0.0 # Center of buoyancy

        # default coefficient values, that can be overriden if data is available
        self.C_mid_section = 0.95  # A_mid_section / (B * T)
        self.C_water_plane = 0.75  # A_water_plane / (B * L)

        self.C_prismatic = self.C_mid_section * self.C_water_plane 

        self.calculate_shape_coefficients()

        # Set wetted surface is given, otherwise estimate empirically
        if S is None:
            self.S = resistance.holtrop_wetted_surface(self)
        else:
            self.S = S

    def calculate_shape_coefficients(self):
        ''' 
        Calculate typical shape coefficients based on the main dimensions of the 
        ship
        '''
        
        self.CB  = self.volume / (self.L * self.B * self.T) # Block coefficient
        self.Asp = 2 * self.T / self.L                      # Double body aspect ratio

    def print(self):
        print('Wetted surface:   ', np.round(self.S, 1), 'm^2')
        print('Longitudonal center of gravity:   ', np.round(self.x_G, 4), 'm')
        print('Longitudonal center of buoyancy:   ', np.round(self.x_B, 4), 'm')
        print('Block coefficient:', np.round(self.CB, 4))
        print('Prismatic coefficient:', np.round(self.C_prismatic, 4))
        print('Mid section coefficient:', np.round(self.C_mid_section, 4))
        print('Water plane coefficient:', np.round(self.C_water_plane, 4))