import numpy as np

class VelocityProfile():
    ''' Model of the target velocity on a route between t = 0.0 to t = t_end '''
    
    def __init__(self, x_end, t_total, t_accel, t_decel):
        self.x_end   = x_end
        self.t_total = t_total

        self.t_accel = t_accel
        self.t_decel0 = t_decel

        self.t_decel_min = 0.5 * self.t_decel0
        self.t_decel_max = 2 * self.t_decel0

        self.allowable_route_error = 0.1
        self.max_solver_iterations = 5

        self.reset_model()
        
        self.t_last = 0.0

    def reset_model(self):
        self.t_decel = float(np.copy(self.t_decel0))

        self.compute_time_parameters()

        u_transit0 = 5 * 0.51444444

        self.set_transit_velocity(u_transit0)

        self.solve_velocity(self.x_end)
        
    def set_transit_velocity(self, u_transit):
        self.u_transit = u_transit
        
        self.compute_time_parameters()
        
    def compute_time_parameters(self):
        self.t_transit = self.t_total - self.t_accel - self.t_decel
        
    def get_target_velocity(self, t):
        t_left = self.t_total - t
        
        if t < self.t_accel:
            u = self.u_transit * t / self.t_accel
        elif t >= self.t_total:
            u = 0.0
        elif t_left <= self.t_decel:
            u = self.u_transit * t_left / self.t_decel
        else:
            u = self.u_transit
            
        return u

    def get_distance_left(self, t_current=0.0):
        if t_current < self.t_accel:
            dt = self.t_accel - t_current

            u_mean = self.get_target_velocity(t_current + dt/2)

            x = u_mean * dt + self.u_transit * self.t_transit + 0.5 * self.u_transit * self.t_decel
        elif t_current < self.t_total - self.t_decel:
            dt = self.t_transit + self.t_accel - t_current

            x = self.u_transit * dt + 0.5 * self.u_transit * self.t_decel
        else:
            dt = self.t_total - t_current

            u_mean = self.get_target_velocity(t_current + dt/2)

            x = u_mean * dt
            
        return x

    def solve_velocity(self, route_length, t_current=0.0):
        iteration = 0
        error = np.inf

        while iteration < self.max_solver_iterations and error > self.allowable_route_error:
            error = self.single_solve_velocity(route_length, t_current=t_current)

            iteration += 1
    
    def single_solve_velocity(self, route_length, t_current=0.0):
        distance_left = self.get_distance_left(t_current=t_current)
            
        u_current = float(np.copy(self.u_transit))
            
        self.set_transit_velocity(u_current * route_length / distance_left)

        return np.abs(distance_left - route_length)

    def single_solve_deceleration(self, route_length, t_current):
        distance_left = self.get_distance_left(t_current=t_current)

        if distance_left > self.allowable_route_error and route_length > 0.0:
            self.t_decel = self.t_decel * distance_left / route_length

            self.t_decel = max(self.t_decel, self.t_decel_min)
            self.t_decel = min(self.t_decel, self.t_decel_max)

            self.compute_time_parameters()

    def update_profile(self, t, x, y):
        if t < self.t_last:
            self.reset_model()
            
        distance_left = np.sqrt((self.x_end - x)**2 + y**2)

        if t < self.t_total - self.t_decel:
            self.single_solve_velocity(distance_left, t_current=t)
        else:
            self.single_solve_deceleration(distance_left, t_current=t)

        self.t_last = float(np.copy(t))