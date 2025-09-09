from numpy import power
import WageningenPropeller as WageningenPropeller

import numpy as np

class PropModel():
    def __init__(self):
        self.front_loading = 0.2
        self.stern_loading = 1 - self.front_loading

        self.front_deduction = 0.25
        self.stern_deduction = 0.1

        self.front_wake = 0.0
        self.stern_wake = 0.1

        self.pd_array = np.array([0.8, 0.9, 1.0, 1.1])

        self.n_pd = len(self.pd_array)

        self.prop_list = []

        self.D = 2.3
        self.AEA0 = 0.75
        self.Z = 4

        self.eta_electric = 0.95
        self.eta_mechannical = 0.95

        for i in range(self.n_pd):
            prop_ = WageningenPropeller.WageningenPropeller(self.D, self.pd_array[i], self.AEA0, self.Z)
            
            self.prop_list.append(prop_)

    def get_power(self, ship_speed, resistance):
        power_array_front = np.zeros(self.n_pd)
        power_array_stern = np.zeros(self.n_pd)

        front_thrust = resistance * self.front_loading * (1 + self.front_deduction)
        stern_thrust = resistance * self.stern_loading * (1 + self.stern_deduction)

        u_front = ship_speed * (1 - self.front_wake)
        u_stern = ship_speed * (1 - self.stern_wake)

        for i in range(self.n_pd):
            n_front = self.prop_list[i].reqRevolutions(front_thrust, u_front)
            n_stern = self.prop_list[i].reqRevolutions(stern_thrust, u_stern)

            power_array_front[i] = self.prop_list[i].power(u_front, n_front)
            power_array_stern[i] = self.prop_list[i].power(u_stern, n_stern)

            if power_array_front[i] < 0:
                power_array_front[i] = 0.0

            if power_array_stern[i] < 0:
                power_array_stern[i] = 0.0

        power_total = (np.min(power_array_stern) + np.min(power_array_front)) / (self.eta_electric * self.eta_mechannical)

        return power_total
