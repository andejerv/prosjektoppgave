import numpy as np
import math
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
plt.style.use('bmh')


'''
Data class for storing hydrostatic data used in the seakeeping model
'''
class Hydrostatics:
        def __init__(self, GM_roll, GM_pitch, waterplane_area):
            self.GM_roll = GM_roll # Roll metacentric height
            self.GM_pitch = GM_pitch # Pitch metacentric height
            self.waterplane_area = waterplane_area


'''
Class for estimating and storing the six degree of freedom masses, moments of inertias and added inertias
'''
class Inertias:
    def __init__(self, ship_dim, physical_constants):
         self.ship_dim = ship_dim
         self.physical_constants = physical_constants
         self.mass = ship_dim.volume * physical_constants.rho

         self.set_default_moments_of_inertia()
         self.set_default_six_dof_added_mass()
    '''
    Estimates the moments of inertia in roll, pitch and yaw based on empirical formulas for the radii of gyration of ships
    '''
    def set_default_moments_of_inertia(self):
         self.I_roll = self.mass*(0.35*self.ship_dim.B)**2 #RoG = 0.35*B in heave
         self.I_pitch = self.mass*(0.25*self.ship_dim.L)**2 #Based on empirical value RoG = 0.25*Lpp in pitch
         self.I_yaw = self.mass*(0.25*self.ship_dim.L)**2 #Based on empirical value RoG = 0.25*Lpp in yaw

    
    '''
    Estimates 6-dof added mass coefficients based on results from "Determination of Added Mass and Inertia Moment of Marine
    Ships Moving in 6 Degrees of Freedom, Do Thanh Sen, Tran Canh Vinh, 2016"
    '''
    def set_default_six_dof_added_mass(self):
        self.added_mass_surge = 0.033 * self.mass
        self.added_mass_sway = 0.986 * self.mass
        self.added_mass_heave = 1.004 * self.mass

        self.added_I_roll = 0.01 * self.mass * self.ship_dim.B**2
        self.added_I_pitch = 0.039 * self.mass * self.ship_dim.L**2
        self.added_I_yaw = 0.045 * self.mass * self.ship_dim.L**2

    '''
    If an maneuvering model is available, this method sets the horizontal (surge, sway, yaw) added mass coefficients from the maneuvering model
    '''
    def set_horizontal_added_mass_from_maneuvering_model(self, man_coeffs):
        self.added_mass_surge = man_coeffs["mx"]
        self.added_mass_sway = man_coeffs["my"]
        self.added_I_yaw = man_coeffs["Jz"]

    def get_total_inertia_surge(self):
        return self.mass + self.added_mass_surge
    
    def get_total_inertia_sway(self):
        return self.mass + self.added_mass_sway
    
    def get_total_inertia_heave(self):
        return self.mass + self.added_mass_heave
    
    def get_total_inertia_roll(self):
        return self.I_roll + self.added_I_roll
    
    def get_total_inertia_pitch(self):
        return self.I_pitch + self.added_I_pitch
    
    def get_total_inertia_yaw(self):
        return self.I_yaw + self.added_I_yaw

class Seakeeping:

    def __init__(self, ship_dim, physical_constants, hydrostatics, inertias, damping_ratios):
        self.ship_dim = ship_dim
        self.physical_constants = physical_constants
        self.hydrostatics = hydrostatics
        self.inertias = inertias

        #Restoring force "spring constants" calcucated analytically based on hydrostatics
        self.g_heave = physical_constants.rho*physical_constants.g*hydrostatics.waterplane_area
        self.g_roll = physical_constants.rho*physical_constants.g*ship_dim.volume*hydrostatics.GM_roll
        self.g_pitch = physical_constants.rho*physical_constants.g*ship_dim.volume*hydrostatics.GM_pitch

        #Tunable parameters to get desired decay response. Damping ratio = 1 gives a critically damped system
        self.damping_ratio_heave = damping_ratios[0]
        self.damping_ratio_roll = damping_ratios[1]
        self.damping_ratio_pitch = damping_ratios[2]

        #Linear damping coefficients
        self.d_heave = 2*self.damping_ratio_heave*math.sqrt(self.g_heave*inertias.get_total_inertia_heave())
        self.d_roll = 2*self.damping_ratio_roll*math.sqrt(self.g_roll*inertias.get_total_inertia_roll())
        self.d_pitch = 2*self.damping_ratio_pitch*math.sqrt(self.g_pitch*inertias.get_total_inertia_pitch())

    
    '''
    Calculates the natural periods of oscillation from a decoupled harmonic oscillator model in heave, roll and pitch 
    '''
    def calculate_natural_periods(self):
        T_heave = 2*math.pi/math.sqrt(self.g_heave/self.inertias.get_total_inertia_heave())
        T_roll = 2*math.pi/math.sqrt(self.g_roll/self.inertias.get_total_inertia_roll())
        T_pitch = 2*math.pi/math.sqrt(self.g_pitch/self.inertias.get_total_inertia_pitch())

        return (T_heave, T_roll, T_pitch)
    
    '''
    Plot the time-domain response of a decay from an initial displacement in heave, roll and pitch
    '''
    def plot_decay_response(self):

        y0 = [1, 1, 1, 0, 0, 0]
        M = np.diag([self.inertias.get_total_inertia_heave(), self.inertias.get_total_inertia_roll(), self.inertias.get_total_inertia_pitch()])
        D = np.diag([self.d_heave, self.d_roll, self.d_pitch])
        G = np.diag([self.g_heave, self.g_roll, self.g_pitch])
        
        #y = [heave, roll, pitch, w, p, q]
        def dydt(t, y):
            x_ddot = -np.dot(np.linalg.inv(M), np.dot(D, y[3:]) + np.dot(G, y[0:3]))
            x_dot = y[3:]
            return np.hstack((x_dot, x_ddot))

        t_end = 30.0
        sol = solve_ivp(dydt, [0, t_end], y0, t_eval=np.linspace(0,t_end,1000))

        plt.figure()
        plt.plot(sol.t, sol.y[0])
        plt.plot(sol.t, sol.y[1])
        plt.plot(sol.t, sol.y[2])
        plt.xlabel('t')
        plt.legend(['Heave', 'Roll', 'Pitch'])
        plt.show()


    def print_seakeeping_data(self):
        (T_heave, T_roll, T_pitch) = self.calculate_natural_periods()
        
        print(f'Heave: Natural period: {T_heave: .2f}s, damping ratio: {self.damping_ratio_heave}')
        print(f'Roll: Natural period: {T_roll: .2f}s, damping ratio: {self.damping_ratio_roll}')
        print(f'Pitch: Natural period: {T_pitch: .2f}s, damping ratio: {self.damping_ratio_pitch}')
         


