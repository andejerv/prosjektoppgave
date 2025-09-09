#!/usr/bin/python3
#
# This script computes gains for the DP Mockup controller based
# on a mass-spring-damper approximation given the FF parameters.
import numpy as np

# Parameters from the feed-forward part of DP Mockup controller
m11 = 7000
m22 = 7000
m33 = 35000

d11 = 330
d22 = 390
d33 = 2500

# Desired natrual frequency and damping ration
omega_n = np.array([0.4, 0.4, 0.4])
zeta = np.array([2, 2, 2])
t_integral = 20 # Time for the integrator to double the P output

m = np.array([m11, m22, m33])
k_p = m*omega_n**2
c = 2*zeta*omega_n*m
k_d = c - np.array([d11, d22, d33])
k_i = k_p/t_integral

print('Tuning parameters:\nP-gain: {}\nD-gain: {}\nI-gain:{}'.format(k_p,k_d,k_i))
