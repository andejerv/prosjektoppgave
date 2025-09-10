import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# Define problem parameters
T = 50       # Number of time steps
dt = 0.1     # Time step duration
L, W = 2, 2  # Length and width of the vessel

# Goal position
x_goal, y_goal = 6, 6  # Target destination

# Create an optimization problem
opti = ca.Opti()

# State variable eta (T x 3) where each row is [x_k, y_k, theta_k]
eta = opti.variable(T, 3)  

# Control variable eta_dot (T-1 x 3) where each row is [vx_k, vy_k, omega_k]
eta_dot = opti.variable(T-1, 3)  

# Normal vectors (T x 2) where each row is [nx_k, ny_k]
n = opti.variable(T, 2)

# Point-on-line variables (T x 2) where each row is [x0_k, y0_k]
p = opti.variable(T, 2)

# Define the line segment (Obstacle)
x_A, y_A = 5, 8  # Endpoint A
x_B, y_B = 5, 3  # Endpoint B

# Normalize normal vectors at each timestep
for k in range(T):
    opti.subject_to(n[k, 0]**2 + n[k, 1]**2 == 1)

# Initial condition: Start at (0,0) with 0 orientation

eta_0 = ca.reshape(ca.MX(np.array([0, 0, 0])),1,3)
opti.subject_to(eta[0, :] == eta_0)

# Motion model: eta[k+1] = eta[k] + eta_dot[k] * dt
for k in range(T-1):
    opti.subject_to(eta[k+1, :] == eta[k, :] + dt * eta_dot[k, :])

# Compute rectangle corners at each time step
for k in range(T):
    x_k, y_k, theta_k = eta[k, 0], eta[k, 1], eta[k, 2]
    cos_theta = ca.cos(theta_k)
    sin_theta = ca.sin(theta_k)

    # Corners of the rectangle
    x_TL = x_k - (L/2)*cos_theta + (W/2)*sin_theta  # Top Left
    y_TL = y_k - (L/2)*sin_theta - (W/2)*cos_theta
    
    x_TR = x_k + (L/2)*cos_theta + (W/2)*sin_theta  # Top Right
    y_TR = y_k + (L/2)*sin_theta - (W/2)*cos_theta
    
    x_BL = x_k - (L/2)*cos_theta - (W/2)*sin_theta  # Bottom Left
    y_BL = y_k - (L/2)*sin_theta + (W/2)*cos_theta
    
    x_BR = x_k + (L/2)*cos_theta - (W/2)*sin_theta  # Bottom Right
    y_BR = y_k + (L/2)*sin_theta + (W/2)*cos_theta

    # Enforce time-dependent hyperplane separation using Point-Normal form
    opti.subject_to(n[k, 0] * (x_TL - p[k, 0]) + n[k, 1] * (y_TL - p[k, 1]) > 0)
    opti.subject_to(n[k, 0] * (x_TR - p[k, 0]) + n[k, 1] * (y_TR - p[k, 1]) > 0)
    opti.subject_to(n[k, 0] * (x_BL - p[k, 0]) + n[k, 1] * (y_BL - p[k, 1]) > 0)
    opti.subject_to(n[k, 0] * (x_BR - p[k, 0]) + n[k, 1] * (y_BR - p[k, 1]) > 0)

    # Ensure obstacle endpoints remain on the other side
    opti.subject_to(n[k, 0] * (x_A - p[k, 0]) + n[k, 1] * (y_A - p[k, 1]) <= 0)
    opti.subject_to(n[k, 0] * (x_B - p[k, 0]) + n[k, 1] * (y_B - p[k, 1]) <= 0)

# Define objective function to move toward goal while minimizing control effort
w1 = 10   # Weight for reaching the goal
w2 = 1    # Weight for minimizing control effort
goal_tracking = ca.sumsqr(eta[:, 0] - x_goal) + ca.sumsqr(eta[:, 1] - y_goal)
control_effort = ca.sumsqr(eta_dot)
opti.minimize(w1 * goal_tracking + w2 * control_effort)

# Solve the optimization problem
opti.solver("ipopt")
sol = opti.solve()

# Extract solutions
eta_sol = sol.value(eta)
eta_dot_sol = sol.value(eta_dot)
n_sol = sol.value(n)
p_sol = sol.value(p)

# Plot results
plt.figure(figsize=(8, 8))

# Plot the line segment (Obstacle)
plt.plot([x_A, x_B], [y_A, y_B], 'ro-', linewidth=3, label="Line Segment (Obstacle)")

# Plot the trajectory
plt.plot(eta_sol[:, 0], eta_sol[:, 1], 'g-o', label="Optimal Trajectory")

# Plot the goal position
plt.scatter(x_goal, y_goal, color='purple', s=100, label="Goal Position", marker="*")

# Plot the dynamic hyperplanes
x_min, y_min = -10, -5
x_max, y_max = 10, 10
x_vals = np.linspace(-1, 7, 100)
for k in range(0, T, 3):  # Show every 3rd time step
    y_vals = (-n_sol[k, 0] * (x_vals - p_sol[k, 0])) / n_sol[k, 1] + p_sol[k, 1]
    # Clip x and y values to the axis bounds
    x_vals_clipped = np.clip(x_vals, x_min, x_max)
    y_vals_clipped = np.clip(y_vals, y_min, y_max)

    # Plot the clipped line
    plt.plot(x_vals_clipped, y_vals_clipped, 'b--', alpha=0.5)
    # plt.plot(x_vals, y_vals, 'b--', alpha=0.5, label=f"Hyperplane at t={k}")

# Plot the vessel's rectangle at each time step
for k in range(0, T, 3):  # Show every 3rd time step
    x_k, y_k, theta_k = eta_sol[k, 0], eta_sol[k, 1], eta_sol[k, 2]
    cos_theta = np.cos(theta_k)
    sin_theta = np.sin(theta_k)

    # Compute rectangle corners
    rect_corners = np.array([
        [x_k - (L/2)*cos_theta + (W/2)*sin_theta, y_k - (L/2)*sin_theta - (W/2)*cos_theta],
        [x_k + (L/2)*cos_theta + (W/2)*sin_theta, y_k + (L/2)*sin_theta - (W/2)*cos_theta],
        [x_k + (L/2)*cos_theta - (W/2)*sin_theta, y_k + (L/2)*sin_theta + (W/2)*cos_theta],
        [x_k - (L/2)*cos_theta - (W/2)*sin_theta, y_k - (L/2)*sin_theta + (W/2)*cos_theta],
        [x_k - (L/2)*cos_theta + (W/2)*sin_theta, y_k - (L/2)*sin_theta - (W/2)*cos_theta]  
    ])
    
    plt.plot(rect_corners[:, 0], rect_corners[:, 1], 'k-', linewidth=1)

# Labels and legend
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.legend()
plt.grid(True)
plt.title("Dynamic Hyperplane Avoidance (Point-Normal Form)")

# Show plot
plt.xlim(-1e10, 1e10)
plt.ylim(-1e10, 1e10)
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.axis('equal')
plt.show()


# NON-VECTOR FORMAT:
# Define problem parameters
# T = 50       # Number of time steps
# dt = 0.1     # Time step duration
# L, W = 2,2  # Length and width of the vessel

# # Goal position
# x_goal, y_goal = 6, 6  # Target destination

# # Create an optimization problem
# opti = ca.Opti()

# # Variables for trajectory
# x = opti.variable(T)  # Vessel center X
# y = opti.variable(T)  # Vessel center Y
# theta = opti.variable(T)  # Vessel orientation (radians)

# # Velocity control inputs
# vx = opti.variable(T-1)
# vy = opti.variable(T-1)
# omega = opti.variable(T-1)  # Angular velocity

# # Define the line segment (Obstacle)
# x_A, y_A = 5, 8  # Endpoint A
# x_B, y_B = 5, 3  # Endpoint B

# # Define time-varying hyperplane parameters (Point-Normal form)
# nx = opti.variable(T)  # Normal vector x-component
# ny = opti.variable(T)  # Normal vector y-component
# x0 = opti.variable(T)  # Point on hyperplane x-coordinate
# y0 = opti.variable(T)  # Point on hyperplane y-coordinate

# # Normalize normal vector at each timestep
# for k in range(T):
#     opti.subject_to(nx[k]**2 + ny[k]**2 == 1)

# # Initial condition: Start at (0,0) with 0 orientation
# opti.subject_to(x[0] == 0)
# opti.subject_to(y[0] == 0)
# opti.subject_to(theta[0] == 0)

# # Motion model: x[k+1] = x[k] + vx[k] * dt
# for k in range(T-1):
#     opti.subject_to(x[k+1] == x[k] + vx[k] * dt)
#     opti.subject_to(y[k+1] == y[k] + vy[k] * dt)
#     opti.subject_to(theta[k+1] == theta[k] + omega[k] * dt)

# # Compute rectangle corners at each time step
# for k in range(T):
#     cos_theta = ca.cos(theta[k])
#     sin_theta = ca.sin(theta[k])

#     # Corners of the rectangle
#     x_TL = x[k] - (L/2)*cos_theta + (W/2)*sin_theta  # Top Left
#     y_TL = y[k] - (L/2)*sin_theta - (W/2)*cos_theta
    
#     x_TR = x[k] + (L/2)*cos_theta + (W/2)*sin_theta  # Top Right
#     y_TR = y[k] + (L/2)*sin_theta - (W/2)*cos_theta
    
#     x_BL = x[k] - (L/2)*cos_theta - (W/2)*sin_theta  # Bottom Left
#     y_BL = y[k] - (L/2)*sin_theta + (W/2)*cos_theta
    
#     x_BR = x[k] + (L/2)*cos_theta - (W/2)*sin_theta  # Bottom Right
#     y_BR = y[k] + (L/2)*sin_theta + (W/2)*cos_theta

#     # Enforce time-dependent hyperplane separation using Point-Normal form
#     opti.subject_to(nx[k] * (x_TL - x0[k]) + ny[k] * (y_TL - y0[k]) > 0)
#     opti.subject_to(nx[k] * (x_TR - x0[k]) + ny[k] * (y_TR - y0[k]) > 0)
#     opti.subject_to(nx[k] * (x_BL - x0[k]) + ny[k] * (y_BL - y0[k]) > 0)
#     opti.subject_to(nx[k] * (x_BR - x0[k]) + ny[k] * (y_BR - y0[k]) > 0)

#     # Ensure obstacle endpoints remain on the other side
#     opti.subject_to(nx[k] * (x_A - x0[k]) + ny[k] * (y_A - y0[k]) <= 0)
#     opti.subject_to(nx[k] * (x_B - x0[k]) + ny[k] * (y_B - y0[k]) <= 0)

# # Define objective function to move toward goal while minimizing control effort
# w1 = 10   # Weight for reaching the goal
# w2 = 1    # Weight for minimizing control effort
# goal_tracking = ca.sumsqr(x - x_goal) + ca.sumsqr(y - y_goal)
# control_effort = ca.sumsqr(vx) + ca.sumsqr(vy) + ca.sumsqr(omega)
# opti.minimize(w1 * goal_tracking + w2 * control_effort)

# # Solve the optimization problem
# opti.solver("ipopt")
# sol = opti.solve()

# # Extract solutions
# x_sol = sol.value(x)
# y_sol = sol.value(y)
# theta_sol = sol.value(theta)
# nx_sol = sol.value(nx)
# ny_sol = sol.value(ny)
# x0_sol = sol.value(x0)
# y0_sol = sol.value(y0)

# # Plot results
# plt.figure(figsize=(8, 8))

# # Plot the line segment (Obstacle)
# plt.plot([x_A, x_B], [y_A, y_B], 'ro-', linewidth=3, label="Line Segment (Obstacle)")

# # Plot the trajectory
# plt.plot(x_sol, y_sol, 'g-o', label="Optimal Trajectory")

# # Plot the goal position
# plt.scatter(x_goal, y_goal, color='purple', s=100, label="Goal Position", marker="*")

# # Plot the dynamic hyperplanes
# x_vals = np.linspace(-1, 7, 100)
# for k in range(0, T, 3):  # Show every 3rd time step
#     y_vals = (-nx_sol[k] * (x_vals - x0_sol[k])) / ny_sol[k] + y0_sol[k]
#     plt.plot(x_vals, y_vals, 'b--', alpha=0.5, label=f"Hyperplane at t={k}")

# # Plot the vessel's rectangle at each time step
# for k in range(0, T, 3):  # Show every 3rd time step
#     cos_theta = np.cos(theta_sol[k])
#     sin_theta = np.sin(theta_sol[k])

#     # Compute rectangle corners
#     rect_corners = np.array([
#         [x_sol[k] - (L/2)*cos_theta + (W/2)*sin_theta, y_sol[k] - (L/2)*sin_theta - (W/2)*cos_theta],
#         [x_sol[k] + (L/2)*cos_theta + (W/2)*sin_theta, y_sol[k] + (L/2)*sin_theta - (W/2)*cos_theta],
#         [x_sol[k] + (L/2)*cos_theta - (W/2)*sin_theta, y_sol[k] + (L/2)*sin_theta + (W/2)*cos_theta],
#         [x_sol[k] - (L/2)*cos_theta - (W/2)*sin_theta, y_sol[k] - (L/2)*sin_theta + (W/2)*cos_theta],
#         [x_sol[k] - (L/2)*cos_theta + (W/2)*sin_theta, y_sol[k] - (L/2)*sin_theta - (W/2)*cos_theta]  
#     ])
    
#     plt.plot(rect_corners[:, 0], rect_corners[:, 1], 'k-', linewidth=1)

# # Labels and legend
# plt.xlabel("X-axis")
# plt.ylabel("Y-axis")
# plt.legend()
# plt.grid(True)
# plt.title("Dynamic Hyperplane Avoidance (Point-Normal Form)")

# # Show plot
# plt.xlim(-1, 8)
# plt.ylim(-1, 8)
# plt.axhline(0, color='grey', lw=0.5, ls='--')
# plt.axvline(0, color='grey', lw=0.5, ls='--')
# plt.axis('equal')
# plt.show()

 


# ## Using ax + by + c to represent the line
# import casadi as ca
# import numpy as np
# import matplotlib.pyplot as plt

# # Define problem parameters
# T = 20       # Number of time steps
# dt = 0.1     # Time step duration
# L, W = 0.5, 0.3  # Length and width of the vessel

# # Goal position
# x_goal, y_goal = 6, 6  # Target destination

# # Create an optimization problem
# opti = ca.Opti()

# # Variables for trajectory
# x = opti.variable(T)  # Vessel center X
# y = opti.variable(T)  # Vessel center Y
# theta = opti.variable(T)  # Vessel orientation (radians)

# # Velocity control inputs
# vx = opti.variable(T-1)
# vy = opti.variable(T-1)
# omega = opti.variable(T-1)  # Angular velocity

# # Define the line segment (Obstacle)
# x_A, y_A = 2, 2  # Endpoint A
# x_B, y_B = 5, 5  # Endpoint B

# # Define time-varying hyperplane parameters
# a = opti.variable(T)
# b = opti.variable(T)
# c = opti.variable(T)

# # Normalize each normal vector (a_k, b_k)
# for k in range(T):
#     opti.subject_to(a[k]**2 + b[k]**2 == 1)

# # Initial condition: Start at (0,0) with 0 orientation
# opti.subject_to(x[0] == 0)
# opti.subject_to(y[0] == 0)
# opti.subject_to(theta[0] == 0)

# # Motion model: x[k+1] = x[k] + vx[k] * dt
# for k in range(T-1):
#     opti.subject_to(x[k+1] == x[k] + vx[k] * dt)
#     opti.subject_to(y[k+1] == y[k] + vy[k] * dt)
#     opti.subject_to(theta[k+1] == theta[k] + omega[k] * dt)

# # Compute rectangle corners at each time step
# for k in range(T):
#     cos_theta = ca.cos(theta[k])
#     sin_theta = ca.sin(theta[k])

#     # Corners of the rectangle
#     x_TL = x[k] - (L/2)*cos_theta + (W/2)*sin_theta  # Top Left
#     y_TL = y[k] - (L/2)*sin_theta - (W/2)*cos_theta
    
#     x_TR = x[k] + (L/2)*cos_theta + (W/2)*sin_theta  # Top Right
#     y_TR = y[k] + (L/2)*sin_theta - (W/2)*cos_theta
    
#     x_BL = x[k] - (L/2)*cos_theta - (W/2)*sin_theta  # Bottom Left
#     y_BL = y[k] - (L/2)*sin_theta + (W/2)*cos_theta
    
#     x_BR = x[k] + (L/2)*cos_theta - (W/2)*sin_theta  # Bottom Right
#     y_BR = y[k] + (L/2)*sin_theta + (W/2)*cos_theta

#     # Enforce time-dependent hyperplane separation
#     opti.subject_to(a[k] * x_TL + b[k] * y_TL + c[k] > 0)
#     opti.subject_to(a[k] * x_TR + b[k] * y_TR + c[k] > 0)
#     opti.subject_to(a[k] * x_BL + b[k] * y_BL + c[k] > 0)
#     opti.subject_to(a[k] * x_BR + b[k] * y_BR + c[k] > 0)

#     # Ensure obstacle endpoints remain on the other side
#     opti.subject_to(a[k] * x_A + b[k] * y_A + c[k] <= 0)
#     opti.subject_to(a[k] * x_B + b[k] * y_B + c[k] <= 0)

# # Define objective function to move toward goal while minimizing control effort
# w1 = 10   # Weight for reaching the goal
# w2 = 1    # Weight for minimizing control effort
# goal_tracking = ca.sumsqr(x - x_goal) + ca.sumsqr(y - y_goal)
# control_effort = ca.sumsqr(vx) + ca.sumsqr(vy) + ca.sumsqr(omega)
# opti.minimize(w1 * goal_tracking + w2 * control_effort)

# # Solve the optimization problem
# opti.solver("ipopt")
# sol = opti.solve()

# # Extract solutions
# x_sol = sol.value(x)
# y_sol = sol.value(y)
# theta_sol = sol.value(theta)
# a_sol = sol.value(a)
# b_sol = sol.value(b)
# c_sol = sol.value(c)

# # Plot results
# plt.figure(figsize=(6, 6))

# # Plot the line segment (Obstacle)
# plt.plot([x_A, x_B], [y_A, y_B], 'ro-', linewidth=3, label="Line Segment (Obstacle)")

# # Plot the trajectory
# plt.plot(x_sol, y_sol, 'g-o', label="Optimal Trajectory")

# # Plot the goal position
# plt.scatter(x_goal, y_goal, color='purple', s=100, label="Goal Position", marker="*")

# # Plot the dynamic hyperplanes
# x_vals = np.linspace(-1, 7, 100)
# for k in range(0, T, 3):  # Show every 3rd time step
#     y_vals = (-a_sol[k] * x_vals - c_sol[k]) / b_sol[k]
#     plt.plot(x_vals, y_vals, 'b--', alpha=0.5)

# # Plot the vessel's rectangle at each time step
# for k in range(0, T, 3):  # Show every 3rd time step
#     cos_theta = np.cos(theta_sol[k])
#     sin_theta = np.sin(theta_sol[k])

#     # Compute rectangle corners
#     rect_corners = np.array([
#         [x_sol[k] - (L/2)*cos_theta + (W/2)*sin_theta, y_sol[k] - (L/2)*sin_theta - (W/2)*cos_theta],
#         [x_sol[k] + (L/2)*cos_theta + (W/2)*sin_theta, y_sol[k] + (L/2)*sin_theta - (W/2)*cos_theta],
#         [x_sol[k] + (L/2)*cos_theta - (W/2)*sin_theta, y_sol[k] + (L/2)*sin_theta + (W/2)*cos_theta],
#         [x_sol[k] - (L/2)*cos_theta - (W/2)*sin_theta, y_sol[k] - (L/2)*sin_theta + (W/2)*cos_theta],
#         [x_sol[k] - (L/2)*cos_theta + (W/2)*sin_theta, y_sol[k] - (L/2)*sin_theta - (W/2)*cos_theta]  # Close the loop
#     ])
    
#     plt.plot(rect_corners[:, 0], rect_corners[:, 1], 'k-', linewidth=1)

# # Labels and legend
# plt.xlim(-1, 7)
# plt.ylim(-1, 7)
# plt.xlabel("X-axis")
# plt.ylabel("Y-axis")
# plt.legend()
# plt.grid(True)
# plt.title("Dynamic Hyperplane Avoidance for a Rectangular Vessel")

# # Show plot
# plt.show()
