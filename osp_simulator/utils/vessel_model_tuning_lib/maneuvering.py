''' 
Implementation of various empirical equations for the model coefficients in the 
MMG model.

A good overview of different empirical equations suitable for the MMG model is 
given in: "Theoretical background and application of MANSIM for ship 
maneuvering simulations"

The following references from this overview paper is currently implemented:

- Lee et. al (1998): "The prediction of ship's manoeuvring performance in the 
  initial design stage"
- Yoshimura et. al (2012): "Hydrodynamic database and manoeuvring prediction 
  method with medium high-speed merchant ships and fishing vessels"

In addition, there is a function that returns experimentally adjusted values for 
the tanker ship KVLCC2

The equations from the different papers are implemented in seperate functions. 
The results from the functions are store in a dictionary with all relevant 
coefficients as keys.
'''

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors as mcolors

class MMGModel:

    def __init__(self, ship_dim, physical_constants):
        self.ship_dim = ship_dim
        self.physical_constants = physical_constants
        self.initialize_coeffs()
        self.set_default_coefficients()


    def initialize_coeffs(self):
        ''' Function that creates a dictionary with coefficients initlized to None'''

        self.mmg_coeffs = {}

        self.mmg_coeffs["Xvv"]   = None
        self.mmg_coeffs["Xvr"]   = None
        self.mmg_coeffs["Xrr"]   = None
        self.mmg_coeffs["Xvvvv"] = None

        self.mmg_coeffs["Yv"]   = None
        self.mmg_coeffs["Yr"]   = None
        self.mmg_coeffs["Yvvv"] = None
        self.mmg_coeffs["Yvrr"] = None
        self.mmg_coeffs["Yvvr"] = None
        self.mmg_coeffs["Yrrr"] = None

        self.mmg_coeffs["Nv"]   = None
        self.mmg_coeffs["Nr"]   = None
        self.mmg_coeffs["Nvvv"] = None
        self.mmg_coeffs["Nvrr"] = None
        self.mmg_coeffs["Nvvr"] = None
        self.mmg_coeffs["Nrrr"] = None

        self.added_mass = {}

        self.added_mass['mx'] = None
        self.added_mass['my'] = None
        self.added_mass['Jz'] = None

    def set_default_coefficients(self):
        ''' 
        Equation that returns the default choice of empirical model. Currently 
        set to the KVLCC2 coeffcients to avoid bugs related to "weird" main 
        dimensions.
        '''
        self.set_kvlcc2_coefficients()

    def set_mansim_added_mass_coefficients(self):
        '''
        Implements the empirical equations for added mass coefficients from the 
        article: "Theoretical background and application of MANSIM for ship 
        maneuvering simulations"

        This consist of three seperate external references for m_x, m_y and J_z 
        (see overview article)
        '''

        m = self.ship_dim.volume / (0.5 * self.ship_dim.L**2 * self.ship_dim.T)

        L_B = self.ship_dim.L / self.ship_dim.B
        T_B = self.ship_dim.T / self.ship_dim.B
        CB = self.ship_dim.CB

        self.added_mass["mx"] = 0.05 * m
        

        self.added_mass["my"] = m * (0.882 - 
                            0.54 * CB * (1 - 1.6 * T_B) - 
                            0.156 * (1 - 0.673 * CB) * L_B + 
                            0.826 * T_B * L_B * (1 - 0.678 * T_B) - 
                            0.638 * T_B * L_B * (1 - 0.669 * T_B) )

        self.added_mass["Jz"] = m * (1/100 * (33 - 
                            76.85 * CB  * (1 - 0.784 * CB) + 
                            3.430 * L_B * (1 - 0.630 * CB) ) )**2

    def set_yoshimura_coefficients(self):
        '''
        Implements the empirical equations for model coefficients from the article 
        Yoshimura et. al (2012): "Hydrodynamic database and manoeuvring prediction 
        method with medium high-speed merchant ships and fishing vessels"
        '''

        # -------------------- Set up working variables ------------------------------------
        CB_LB = self.ship_dim.CB / (self.ship_dim.L / self.ship_dim.B)
        L_B = self.ship_dim.L / self.ship_dim.B

        # -------------------- Surge coefficients -----------------------------------------
        self.mmg_coeffs["Xvv"]   =  1.15 * CB_LB - 0.18
        self.mmg_coeffs["Xvr"]   =  self.added_mass["my"] - 1.91 * CB_LB + 0.08
        self.mmg_coeffs["Xrr"]   = -0.085 * CB_LB + 0.008 - self.ship_dim.x_G * self.added_mass["my"]
        self.mmg_coeffs["Xvvvv"] = -6.68 * CB_LB + 1.1

        # -------------------- Sway coefficients -----------------------------------------
        self.mmg_coeffs["Yv"]   = -(0.5 * np.pi * self.ship_dim.Asp + 1.4 * CB_LB)
        self.mmg_coeffs["Yr"]   =  self.added_mass["mx"] + 0.5 * CB_LB
        self.mmg_coeffs["Yvvr"] = -0.75
        self.mmg_coeffs["Yvrr"] = -(0.26 * (1 - self.ship_dim.CB) * L_B + 0.11)
        self.mmg_coeffs["Yvvv"] = -0.185 * L_B + 0.48
        self.mmg_coeffs["Yrrr"] = -0.051

        # -------------------- Yaw coefficients -----------------------------------------
        self.mmg_coeffs["Nv"]   = -self.ship_dim.Asp
        self.mmg_coeffs["Nr"]   = -self.ship_dim.Asp * (0.54 + self.ship_dim.Asp )
        self.mmg_coeffs["Nvvr"] =  1.55 * CB_LB - 0.76
        self.mmg_coeffs["Nvrr"] = -0.075 * (1 - self.ship_dim.CB) * L_B - 0.098
        self.mmg_coeffs["Nvvv"] = -(-0.69 * self.ship_dim.CB + 0.66)
        self.mmg_coeffs["Nrrr"] =  0.25 * CB_LB - 0.056

    def set_lee_mmg_coefficients(self):
        ''' 
        Implements the empirical equations for model coefficients from the article 
        Lee et. al (1998): "The prediction of ship's manoeuvring performance in the 
        initial design stage" 
        
        WARNING: Some of the coefficients from this method gets unreasnoably high 
        values for certain main dimensions.
        '''

        # -------------------- Set up working variables ------------------------------------

        CB_LB = self.ship_dim.CB / (self.ship_dim.L / self.ship_dim.B)
        CB_BT = self.ship_dim.CB * self.ship_dim.B / self.ship_dim.T
        L_T   = self.ship_dim.L / self.ship_dim.T
        T_L   = self.ship_dim.T / self.ship_dim.L

        m = self.ship_dim.volume / (0.5 * self.ship_dim.L**2 * self.ship_dim.T)

        # -------------------- Surge coefficients -----------------------------------------
        self.mmg_coeffs["Xvv"]   = 0.0014 - 0.1975 * self.ship_dim.T * ( (1 - self.ship_dim.CB) / self.ship_dim.B) * L_T
        self.mmg_coeffs["Xvr"] = ( m + 0.1176 * self.added_mass["my"] * ( 0.5 + self.ship_dim.CB) )
        self.mmg_coeffs["Xrr"] = (-0.0027 + 0.0076 * self.ship_dim.CB * self.ship_dim.T / self.ship_dim.B) * L_T
        self.mmg_coeffs["Xvvvv"] = 0.0

        # -------------------- Sway coefficients -----------------------------------------
        self.mmg_coeffs["Yv"]   = -0.4545 + 0.065 * CB_BT
        self.mmg_coeffs["Yr"]   = -0.1150 * CB_BT + 0.002400 * L_T
        self.mmg_coeffs["Yvvr"] =  0.1234 * CB_LB - 0.001452 * L_T
        self.mmg_coeffs["Yvrr"] = -(0.4346 * (1 - self.ship_dim.CB) * self.ship_dim.T / self.ship_dim.B) * L_T
        self.mmg_coeffs["Yvvv"] = -0.185 * self.ship_dim.L / self.ship_dim.B + 0.48
        self.mmg_coeffs["Yrrr"] = -0.0233 * CB_LB + 0.0063 * L_T

        # -------------------- Yaw coefficients -----------------------------------------
        self.mmg_coeffs["Nv"]   = (-0.23 * T_L + 0.0059) * L_T
        self.mmg_coeffs["Nr"]   = (-0.003724 + 0.10446 * T_L - 1.393 * T_L**2) * L_T
        self.mmg_coeffs["Nvvr"] = (-1.722 + 22.997 * CB_LB - 77.268 * CB_LB**2) * L_T
        self.mmg_coeffs["Nvrr"] = (-0.0005 + 0.00594 * self.ship_dim.CB * self.ship_dim.T / self.ship_dim.B) * self.ship_dim.L / self.ship_dim.T
        self.mmg_coeffs["Nvvv"] = (0.348 - 0.5283 * (1 - self.ship_dim.CB)* self.ship_dim.T / self.ship_dim.L) * L_T
        self.mmg_coeffs["Nrrr"] =  -0.0572 * L_T + 0.03 * self.ship_dim.CB

    def set_kvlcc2_coefficients(self):
        ''' 
        Returns the coefficients for the tanker KVLCC2, tuned based on physical 
        experiments. 

        The values are from the article "Introduction of MMG standard method for 
        ship maneuvering predictions" 
        '''

        m = self.ship_dim.volume / (0.5  * self.ship_dim.L**2 * self.ship_dim.T)

        self.added_mass["mx"] = 0.022 * (0.5 * self.physical_constants.rho * self.ship_dim.L**2 * self.ship_dim.T)
        self.added_mass["my"] = 0.223 * (0.5 * self.physical_constants.rho * self.ship_dim.L**2 * self.ship_dim.T)
        self.added_mass["Jz"] = 0.011 * (0.5 * self.physical_constants.rho * self.ship_dim.L**4 * self.ship_dim.T)

        self.mmg_coeffs["Xvv"]   = -0.040
        self.mmg_coeffs["Xvr"]   =  0.002
        self.mmg_coeffs["Xrr"]   =  0.011
        self.mmg_coeffs["Xvvvv"] =  0.771

        # -------------------- Sway coefficients -----------------------------------------
        self.mmg_coeffs["Yv"]   = -0.315
        self.mmg_coeffs["Yr"]   =  0.083
        self.mmg_coeffs["Yvvr"] =  0.379
        self.mmg_coeffs["Yvrr"] = -0.391
        self.mmg_coeffs["Yvvv"] = -1.607
        self.mmg_coeffs["Yrrr"] =  0.008

        # -------------------- Yaw coefficients -----------------------------------------
        self.mmg_coeffs["Nv"]   = -0.137
        self.mmg_coeffs["Nr"]   = -0.049
        self.mmg_coeffs["Nvvr"] = -0.294
        self.mmg_coeffs["Nvrr"] =  0.055
        self.mmg_coeffs["Nvvv"] = -0.030
        self.mmg_coeffs["Nrrr"] = -0.013

        return self.mmg_coeffs

    def stabilize_coefficients(self, U_stable, x_cp=None):
        ''' 
        Helper function that modifies maneuvering coefficients such that the mode achive stable behaviour at a target speed.

        The reason for this function is that many of the empirical models above results in unstable models for 
        vessel typical for Zeabuz (i.e., mA2). Whether this is physical correct or not is difficult to know without 
        CFD and/or physical testing. A guaranteed stable model is useful in some situations, such as for testing purposes. 

        This function is therefore inteded to help set up models when you need are in need of a stable model

        The stability cireteria is based on a linear model analysis (see TMR4220 - Ship Manoeuvering lecture notes for details)
        '''

        if not(x_cp is None):
            self.mmg_coeffs['Nv'] = x_cp * self.mmg_coeffs['Yv'] # Force center of pressure to be located at a target location

        
        m_prime = self.ship_dim.volume / (0.5 * self.ship_dim.L * self.ship_dim.T)

        self.mmg_coeffs['Nr'] = (self.mmg_coeffs['Nv'] / self.mmg_coeffs['Yv']) * (self.mmg_coeffs['Yr'] - m_prime / U_stable**2)

    def check_linear_stability(self, U):
        '''Function that calculates the linear stability of the model at speed U'''
        
        M_prime = self.ship_dim.volume / (0.5 * self.ship_dim.T * self.ship_dim.L * U**2)
        
        Ir = self.mmg_coeffs['Nr'] / (self.mmg_coeffs['Yr'] - M_prime)
        Iv = self.mmg_coeffs['Nv'] / self.mmg_coeffs['Yv']
        
        return Ir - Iv
    
    def plot_model_parameters(self):
        fig = plt.figure()
        ax1 = fig.add_subplot(311)
        ax2 = fig.add_subplot(312)
        ax3 = fig.add_subplot(313)

        ax_list = [ax1, ax2, ax3]

        for i_a in range(3):
            ax = ax_list[i_a]
            plt.sca(ax)
            
            if i_a == 0:
                keys = ["Xvv", "Xrr", "Xvr", "Xvvvv"]
            elif i_a == 1:
                keys = ["Yv", "Yr", "Yvvr", "Yvrr", "Yvvv"]
            elif i_a == 2:
                keys = ["Nv", "Nr", "Nvvr", "Nvrr", "Nvvv"]

            n = len(keys)

            x0 = np.linspace(1, n, n)

            width = 0.75

            for i in range(len(keys)):
                key = keys[i]

                x = x0[i]

                height = self.mmg_coeffs[key]

                plt.bar(x, height, width=width, color='blue')

            plt.xticks(x0)
            
            labels = [item.get_text() for item in ax.get_xticklabels()]
            labels = keys

            ax.set_xticklabels(labels)
