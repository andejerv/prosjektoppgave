from pythonfmu import Fmi2Slave, Fmi2Causality, Real
from docking_algorithms.auto_docking import AutoDocking
import numpy as np


# Create FMU by running: pythonfmu build -f path/to/extended_dp_docking_fmu.py in terminal

class DockingControllerFmu(Fmi2Slave):
    def __init__(self, **kwargs) -> None:   # kwargs: keyword arguments; extra parameters passed when creating instance - forwarded to parent class ( Fmi2Slave)
        # define state variable for model (internal state that changes over time)
        super().__init__(**kwargs)
        self.step_size = 0.1
        self.current_time = 0

        # inputs from other FMUs
        self.eta = np.array([0, 0, 0, 0, 0, 0])
        for i in range(len(self.eta)):
            setattr(self, f"eta[{i}]", self.eta[i])
            self.register_variable(Real(f"eta[{i}]", self.eta[i], causality=Fmi2Causality.input))

        self.eta_dot = np.array([0, 0, 0, 0, 0, 0])
        for i in range(len(self.eta_dot)):
            setattr(self, f"eta_dot[{i}]", self.eta_dot[i])
            self.register_variable(Real(f"eta_dot[{i}]", self.eta_dot[i], causality=Fmi2Causality.input))

        self.acc = np.array([0, 0, 0, 0, 0, 0])
        for i in range(len(self.acc)):
            setattr(self, f"acc[{i}]", self.acc[i])
            self.register_variable(Real(f"acc[{i}]", self.acc[i], causality=Fmi2Causality.input))

        self.eta_3dof = np.array([self.eta[0], self.eta[1], self.eta[5]])
        self.eta_dot_3dof = np.array([self.eta_dot[0], self.eta_dot[1], self.eta_dot[5]])
        self.eta_ddot_3dof = np.array([self.acc[0], self.acc[1], self.acc[5]])
        self.extended_dp = AutoDocking(t0=self.current_time, dt=self.step_size, init_pose=self.eta_3dof, init_velocity=self.eta_3dof)

        # outputs to other FMUs
        self.ref_forces = np.array([0, 0, 0])
        for i in range(len(self.ref_forces)):
            setattr(self, f"ref_forces[{i}]", self.ref_forces[i])
            self.register_variable(Real(f"ref_forces[{i}]", causality=Fmi2Causality.output))



    def do_step(self, current_time: float, step_size: float) -> bool: # Defines what happens during each time step of simulation
        # update state of model

        self.step_size = step_size
        self.current_time = current_time
        # get attribute from other FMU
        for i in range(len(self.eta)):
            getattr(self, f"eta[{i}]", self.eta[i])
        
        for i in range(len(self.eta_dot)):
            getattr(self, f"eta_dot[{i}]", self.eta_dot[i])

        for i in range(len(self.acc)):
            getattr(self, f"acc[{i}]", self.acc[i])

        eta_3dof = np.array([self.eta[0], self.eta[1], self.eta[5]])
        eta_dot_3dof = np.array([self.eta_dot[0], self.eta_dot[1], self.eta_dot[5]])
        eta_ddot_3dof = np.array([self.acc[0], self.acc[1], self.acc[5]])

        self.ref_forces = self.extended_dp.run(dt=self.step_size, t0=self.current_time, eta=eta_3dof, eta_dot=eta_dot_3dof)    # reference control forces

        # set attribute for other FMUs to use
        for i in range(len(self.ref_forces)):
            setattr(self, f"ref_forces[{i}]", self.ref_forces[i])

        # Could possibly try: self.generalized_force[0], self.generalized_force[1], self.generalized_force[2] = control_forces instead of loop above


        return True # indicate simulation step successful
        # current_time: current simulation time at start of the step
        # step_size: indicates amount of time passed since last do_step call
