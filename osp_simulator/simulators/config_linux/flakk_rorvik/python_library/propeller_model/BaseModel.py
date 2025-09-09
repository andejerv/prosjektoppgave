import numpy as np

import scipy.optimize as optimize

class BaseModel():
	def __init__(self, D, PD, AEA0, Z, rho=1025.9):
		self.D    = D    # Diameter
		self.PD   = PD   # Pitch/Diameter
		self.AEA0 = AEA0 # Expanded blade area/disk area
		self.Z    = Z    # Number of blades

		self.rho = rho

		self.Ap = np.pi * (self.D / 2)**2

		self.setHullData(0.0, 0)

	def setHullData(self, w, t):
		self.w = w
		self.t = t

		self.eta_h = (1-self.t)/(1-self.w)

	def incomingVelocity(self, Uship):
		return Uship * (1 - self.w)

	def findMaxJ(self):
		return optimize.newton(self.KT, 0.01)

	def thrust(self, U, n):
		U_eff = self.incomingVelocity(U)

		J = self.advanceRatio(U_eff, n)

		KT = self.KT(J)

		T =  KT * self.rho * n**2 * self.D**4

		return T

	def torque(self, U, n):
		U_eff = self.incomingVelocity(U)

		J = self.advanceRatio(U_eff, n)

		KQ = self.KQ(J)

		Q = KQ * self.rho * n**2 * self.D**5

		return Q

	def power(self, U, n):
		Q = self.torque(U, n)
		P = 2 * np.pi * n * Q

		return P

	def advanceRatio(self, U, n):
		J = U / (n * self.D)

		return J

	def efficiency(self, U, n):
		T = self.thrust(U, n)
		P = self.power(U, n)

		eta0 = T * U / P

		eta = eta0 * self.eta_h

		return eta

	def thrustCoefficient(self, U, n):
		T     = self.thrust(U, n)
		U_eff = self.incomingVelocity(U)

		CT = T / (0.5 * self.rho * self.Ap * U_eff**2)

		return CT

	def propellerDiskInducedVelocity(self, U, n):
		CT = self.thrustCoefficient(U, n)
		U_eff = self.incomingVelocity(U)

		UA = (np.sqrt(1 + CT) - 1) * U_eff

		return 0.5 * UA

	def reqRevolutionsObjFunc(self, n, U, T_req):
		T = self.thrust(U, n)

		return (T - T_req) / T_req

	def reqRevolutions(self, T_req, Uship):
		if T_req <= 0.0:
			n_req = 0.0
		else:
			U = self.incomingVelocity(Uship)

			n_min = U / (self.J_max * self.D)

			n_req = optimize.newton(self.reqRevolutionsObjFunc, n_min*1.2, args=(U, T_req))

		return n_req

	def changePitch(self, PD):
		self.PD = float(np.copy(PD))

		self.J_max = self.findMaxJ()
