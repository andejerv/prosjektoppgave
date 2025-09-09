import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
from enum import Enum
plt.style.use('bmh')


class PropellerGeometry(Enum):
    WAGENINGEN_B3_65_PD1 = 1
    WAGENINGEN_B4_55_PD1 = 2
    WAGENINGEN_B4_70_PD08 = 3
    WAGENINGEN_B4_70_PD1 = 4
    WAGENINGEN_B4_70_PD14 = 5

class FourQuadrantPropeller():
    def __init__(self, propeller_geometry: PropellerGeometry, propeller_diameter, pitch_scaling_factor = 1.0):
        self.rho = 1025.9
        self.propeller_diameter = propeller_diameter
        self.propeller_geometry = propeller_geometry
        
        # Read four quadrant fourier series from csv
        dir_path = os.path.dirname(os.path.realpath(__file__))
        four_quadrant_data_file = os.path.join(dir_path, 'propeller_data', f'4Q_{propeller_geometry.name}.csv')
        data = pd.read_csv(four_quadrant_data_file, header=None).to_numpy()
        self.n_coeff = data.shape[1]
        self.AT = data[0,:]*pitch_scaling_factor
        self.BT = data[1,:]*pitch_scaling_factor
        self.AQ = data[2,:]*pitch_scaling_factor
        self.BQ = data[3,:]*pitch_scaling_factor

    def get_CT(self, beta):
        CT = 0
        for k in range(self.n_coeff):
            CT = CT + self.AT[k]*math.cos(beta*k) + self.BT[k]*math.sin(beta*k)
        
        return CT
    

    def get_CQ(self, beta):
        CQ = 0
        for k in range(self.n_coeff):
            CQ = CQ + self.AQ[k]*math.cos(beta*k) + self.BQ[k]*math.sin(beta*k)
        
        return CQ
    
    def get_thrust(self, advance_vel, rpm):
        n = rpm / 60.
        beta = math.atan2(advance_vel, 0.7*math.pi*n*self.propeller_diameter)
        CT = self.get_CT(beta)
        thrust = 0.5* self.rho * CT * (advance_vel**2 + (0.7 * math.pi * n * self.propeller_diameter)**2) * (math.pi/4) * self.propeller_diameter**2
        return thrust
    
    def get_torque(self, advance_vel, rpm):
        n = rpm / 60.
        beta = math.atan2(advance_vel, 0.7*math.pi*n*self.propeller_diameter)
        CQ = self.get_CQ(beta)
        torque = -0.5* self.rho * CQ * (advance_vel**2 + (0.7 * math.pi * n * self.propeller_diameter)**2) * (math.pi/4) * self.propeller_diameter**3
        return torque
    
    def get_power(self, adance_vel, rpm):
        return 2*math.pi*(rpm/60)*self.get_torque(adance_vel, rpm)
    
    def four_quadrant_plot(self):
        n_beta = 100
        betas = np.linspace(-math.pi, math.pi, n_beta)
        CT = np.zeros(n_beta)
        CQ = np.zeros(n_beta)
        for i in range(n_beta):
            CT[i] = self.get_CT(betas[i])
            CQ[i] = self.get_CQ(betas[i])

        plt.figure()
        plt.plot(betas*180/math.pi, CT)
        plt.plot(betas*180/math.pi, 10*CQ)
        plt.xlabel('beta [deg]')
        plt.legend(['CT', '10CQ',])
        plt.title('Four quadrant propeller coefficients')

    def get_KT0(self):
        CT = self.get_CT(0.0)
        return CT*(math.pi/8.0)*0.7**2*math.pi**2

    
    def plot_thrust_torque_power_vs_rpm(self, rpm_list, advance_vel):
        n_points = len(rpm_list)
        T = np.zeros(n_points)
        Q = np.zeros(n_points)
        P = np.zeros(n_points)

        for i in range(n_points):
            T[i] = self.get_thrust(advance_vel, rpm_list[i])
            Q[i] = self.get_torque(advance_vel, rpm_list[i])
            P[i] = self.get_power(advance_vel, rpm_list[i])
            

        plt.figure()
        thrust_ax = plt.gca()
        plt.plot(rpm_list, T/1e3)
        plt.xlabel('RPM')
        plt.ylabel('Thrust [kN]')
        plt.title('Thrust vs rpm')

        plt.figure()
        torque_ax = plt.gca()
        plt.plot(rpm_list, Q/1e3)
        plt.xlabel('RPM')
        plt.ylabel('Torque [kNm]')
        plt.title('Torque vs rpm')

        plt.figure()
        power_ax = plt.gca()
        plt.plot(rpm_list, P/1e3)
        plt.xlabel('RPM')
        plt.ylabel('Shaft power [kW]')
        plt.title('Power vs rpm')

        return (thrust_ax, torque_ax, power_ax)

    def plot_thrust_torque_power_vs_speed(self, advance_vel_list, rpm):
        n_points = len(advance_vel_list)
        T = np.zeros(n_points)
        Q = np.zeros(n_points)
        P = np.zeros(n_points)

        for i in range(n_points):
            T[i] = self.get_thrust(advance_vel_list[i], rpm)
            Q[i] = self.get_torque(advance_vel_list[i], rpm)
            P[i] = self.get_power(advance_vel_list[i], rpm)
            

        plt.figure()
        thrust_ax = plt.gca()
        plt.plot(advance_vel_list, T/1e3)
        plt.xlabel('Advance velocity [m/s]')
        plt.ylabel('Thrust [kN]')
        plt.title('Thrust vs speed')

        plt.figure()
        torque_ax = plt.gca()
        plt.plot(advance_vel_list, Q/1e3)
        plt.xlabel('Advance velocity [m/s]')
        plt.ylabel('Torque [kNm]')
        plt.title('Torque vs speed')

        plt.figure()
        power_ax = plt.gca()
        plt.plot(advance_vel_list, P/1e3)
        plt.xlabel('Advance velocity [m/s]')
        plt.ylabel('Shaft power [kW]')
        plt.title('Power vs speed')
        
        return (thrust_ax, torque_ax, power_ax)
    
def transform_four_quadrant_coefficient_file(old_file, new_file):
    '''
    Transform data from source paper, where the four quadrant coefficients
    are listed to yield 100*CT and -1000*CQ.
    '''

    # Read old data from csv
    data = pd.read_csv(old_file, header=None).to_numpy()
    data[:, 0] = data[:, 0]/100.0
    data[:, 1] = data[:, 1]/100.0
    data[:, 2] = data[:, 2]/-1000.0
    data[:, 3] = data[:, 3]/-1000.0

    new_data = pd.DataFrame(data)
    new_data.to_csv(new_file, index=False, header=False, float_format="%.4e")


def transpose_csv(old_file, new_file):
    data = pd.read_csv(old_file, header=None).transpose()
    data.to_csv(new_file, index=False, header=False, float_format="%.4e")