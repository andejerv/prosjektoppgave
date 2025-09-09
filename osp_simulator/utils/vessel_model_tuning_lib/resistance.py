''' 
Implementation of various empirical models for straight ahead resistance 
components. 

'''
import numpy as np
import scipy.optimize as optimize

class Resistance:
    ''' 
    Class that implements the resistance model (same as the FMU implementation),
    '''
    
    def __init__(self, ship_dim, physical_constants):
        self.ship_dim = ship_dim
        self.physical_constants = physical_constants
        self.k = 0.0 # Shape factor
        self.CD_lateral = 0.0 # Additional drag due to viscous effects, such as submerged transom stern
        self.CR_m = 0.0 # Wave resistance polynomial mantissa CR = CR_m * Fr ^CP_p
        self.CR_p = 0.0
        
    def get_resistance(self, u):
        Fr = u / np.sqrt(self.ship_dim.L * self.physical_constants.g)
        
        CF = CF_ITTC(self.ship_dim, u, self.physical_constants.nu)
        
        CR = CR_wave_polynomial(Fr, self.CR_m, self.CR_p)
        
        #Base drag as calculated in FMU
        Cbd = self.CD_lateral * (self.ship_dim.T * self.ship_dim.L / self.ship_dim.S)
       
        CT = CF * (1 + self.k) + CR + Cbd
        
        return CT * 0.5 * self.physical_constants.rho * self.ship_dim.S * u**2

def CR_wave_polynomial(Fr, m, p):
    ''' Simplified polynomial model for the wave resistance '''
    
    return m * Fr**p

def fit_wave_polynomial(Fr, CR):
    ''' Function that fits wave resistance data to a polynomial model'''

    popt, pcov = optimize.curve_fit(CR_wave_polynomial, Fr, CR)
    m = popt[0]
    p = popt[1]

    return (m, p)

def CF_ITTC(ship_dim, U, nu):
    ''' Frictional resistance coefficient from the ITTC'''

    Re = max(1000, ship_dim.L * U / nu)

    CF = 0.075 / (np.log10(Re) - 2)**2

    return CF

def marintek_shape_factor(ship_dim):
    ''' 
    Empirical equation for the visocus shape factor suggested by MARINTEK 
    (now SINTEF Ocean)
    '''

    phi = ship_dim.CB * np.sqrt(ship_dim.B * 2 * ship_dim.T) / ship_dim.L

    k = 0.6 * phi + 145 * phi**3.5
    
    return k

def holtrop_wetted_surface(ship_dim):
    '''
    Estimates the wetted surface of a ship based on the main dimensions, as 
    specified in the Holtrop method:
    Holtrop et al. 1987: "An Approximate power prediction method"
    '''

    L   = ship_dim.L
    B   = ship_dim.B
    T   = ship_dim.T
    CB  = ship_dim.CB
    CM  = ship_dim.C_mid_section
    CWP = ship_dim.C_water_plane

    S = L * (2 *T + B) * np.sqrt(CM) * (
        0.453 
        + 0.4425 * CB 
        - 0.2862 * CM
        - 0.003467 * B / T
        + 0.3696 * CWP 
    )

    return S

def holtrop_wave_resistance(ship_dim, U, g, rho):
    '''
    Calculates the wave resistacne according to the Holtrop 87 empirical model:
    Holtrop et al. 1987: "An Approximate power prediction method"
    '''

    if U > 0.0:
        Fr = U / np.sqrt(ship_dim.L * g)

        lcb = ship_dim.x_B / ship_dim.L
        L_R = ship_dim.L * (1 - ship_dim.C_prismatic
                            + 0.06 * ship_dim.C_prismatic * lcb / (4 *ship_dim.C_prismatic - 1) )

        i_E = 1 + 89 * np.exp( 
                            - (ship_dim.L / ship_dim.B)**0.80856 
                            * (1 - ship_dim.C_water_plane)**0.30484
                            * (1 - ship_dim.C_prismatic - 0.0225 * lcb)**0.6367
                            * (L_R / ship_dim.B)**0.34574
                            * (100 * ship_dim.volume / ship_dim.L**3)**0.16302 
                            )

        B_L = ship_dim.B / ship_dim.L

        if B_L < 0.11:
            c7 = 0.229577 * (ship_dim.B / ship_dim.L)**(1.0/3)
        elif B_L < 0.25:
            c7 = ship_dim.B / ship_dim.L
        else:
            c7 = 0.5 - 0.0625 * ship_dim.L / ship_dim.B

        c1 = 2223105 * c7**(3.78613) * (ship_dim.T / ship_dim.B)**1.07961 * (90 - i_E)**(-1.37565) 
        c2 = 1.0 # Actually a variable dependent on the bulbous bow geometry. Current implementation assumed no bulb (this is standard practice)
        c5 = 1.0 # Actually a variable that takes into account transom stern. Current implementation assumes no transom 

        L_B = ship_dim.L / ship_dim.B

        if L_B < 12:
            lam = 1.446 * ship_dim.C_prismatic - 0.03 * L_B
        else:
            lam = 1.446 * ship_dim.C_prismatic - 36

        d = -0.9

        if ship_dim.C_prismatic < 0.8:
            c16 = (
                8.079810 * ship_dim.C_prismatic
                - 13.86730 * ship_dim.C_prismatic**2
                + 6.984388 * ship_dim.C_prismatic**3
            )
        else:
            c16 = 1.73014 - 0.7067 * ship_dim.C_prismatic

        m1 = (
            0.0140407 * ship_dim.L / ship_dim.T
            - 1.75254 * ship_dim.volume**(1.0/3) / ship_dim.L
            - 4.79323 * ship_dim.B / ship_dim.L
            - c16
        )

        L_3_nabla = ship_dim.L**3 / ship_dim.volume

        if L_3_nabla < 512:
            c15 = -1.69385
        elif L_3_nabla < 1727:
            c15 = -1.69385 + (ship_dim.L / ship_dim.volume**(1.0/3) - 8.0) / 2.36
        else:
            c15 = 0.0

        m2 = c15 * ship_dim.C_prismatic**2 * np.exp(-0.1 * Fr**(-2) )

        R_W = c1 * c2 * c5 * ship_dim.volume * rho * g * np.exp(m1 * Fr**d + m2 * np.cos(lam * Fr**(-2)))

        CR = R_W / (0.5 * rho * ship_dim.S * U**2)
    else:
        CR = 0.0
    
    return CR

def hollenbach_wave_resistance(ship_dim, U, g):
    '''
    Calculates the wave resistance according to the Hollenbach 98 empirical model:
    Hollenbach et. al (1998): "Estimating Resistance and Propulsion for 
    Single-Screw and Twin-Screw Ships"

    The implementaiton has neglected all terms related to rudder and propeller 
    geoemtry, and assumed "mean values"
    '''

    Fr = U / np.sqrt(ship_dim.L * g)

    Dp = 0.75 * ship_dim.T # Rough estimate to be used as input to the calculations

    f1 = 0.17
    f2 = 0.20
    f3 = 0.60

    g1 =  0.642
    g2 = -0.635
    g3 =  0.150

    Fr_min = min(f1, f1 + f2 * (f3 - ship_dim.CB))
    Fr_max = g1 + g2 * ship_dim.CB + g3 * ship_dim.CB**2

    if Fr >= Fr_min and Fr <= Fr_max:

        a1 = -0.3382
        a2 =  0.8086
        a6 =  0.0146

        b11 = -0.57424
        b12 =  13.3893
        b13 =  90.5960

        b21 =  4.6614
        b22 = -39.721
        b23 = -351.483

        b31 = -1.14215
        b32 = -12.3296
        b33 =  459.254

        d1 =  0.854
        d2 = -1.228
        d3 =  0.497

        e1 =  2.1701
        e2 = -0.1602

        CR_standard = (         b11 + b12 * Fr + b13 * Fr**2
            + ship_dim.CB**1 * (b21 + b22 * Fr + b23 * Fr**2)
            + ship_dim.CB**2 * (b31 + b32 * Fr + b33 * Fr**2)
        ) / 10

        Fr_critical = d1 + d2 * ship_dim.CB + d3 * ship_dim.CB**2

        c1 = Fr / Fr_critical

        CR_critical = max(1.0, (Fr / Fr_critical)**c1 )

        k_L = e1 * ship_dim.L**e2

        CR = CR_standard * CR_critical * k_L * (
              (ship_dim.T / ship_dim.B)**a1
            * (ship_dim.B / ship_dim.L)**a2
            * (Dp / ship_dim.T)**a6
        )

        CR *= (ship_dim.T * ship_dim.B) / ship_dim.S
    else:
        CR = 0.0

    return CR