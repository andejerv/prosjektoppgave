import numpy as np
import plotly.graph_objects as go
import pytest
import os

from docking_algorithms.utils.types import Waypoint
from docking_algorithms.guidance.guidance import Guidance
from generate_results.read_write_files import save_fig_plotly

import matplotlib.pyplot as plt
import plotly.graph_objects as go

import pytest
import numpy as np


def test_speed_law_sigmoid():
    # Constants
    deceleration_threshold = 14  # Example threshold value
    lambda_wp = 8  # Example lambda value for steepness of the curve
    curr_wp_speed = 0.5  # Speed at the current waypoint
    prev_wp_speed = 1.5  # Speed at the previous waypoint

    dist_left_to_curr_wp = np.linspace(deceleration_threshold + 30, 0, 100)
    computed_speeds = []

    # Calculate speed based on the given distance
    for i, d in enumerate(dist_left_to_curr_wp):
    #     if d <= deceleration_threshold:
            # speed_factor = 1 / (1 + np.exp(-lambda_wp * (d / deceleration_threshold - 0.5)))
        speed_factor = 1 / (1 + np.exp(-lambda_wp * (d - deceleration_threshold) / deceleration_threshold))
        # else:
            # speed_factor = 1  # In case the distance exceeds the threshold, speed is unchanged (or could be handled differently)
    
        speed = curr_wp_speed + (prev_wp_speed - curr_wp_speed) * speed_factor
        computed_speeds.append(speed)
    # speed_factor = 1 / (1 + np.exp(-lambda_wp * (14 / deceleration_threshold - 0.5)))
    # speed = curr_wp_speed + (prev_wp_speed - curr_wp_speed) * speed_factor
    
    b = 1

    # Create interactive plot
    fig = go.Figure()

    # Add speed curve
    fig.add_trace(go.Scatter(
        x=dist_left_to_curr_wp,
        y=computed_speeds,
        mode='lines',
        name=f'λ = {lambda_wp}',
        line=dict(color='blue')
    ))

    # Mark start and end speeds
    fig.add_trace(go.Scatter(
        x=[dist_left_to_curr_wp[0]],
        y=[computed_speeds[0]],
        mode='markers',
        marker=dict(color='purple', size=8),
        name="Start Speed"
    ))
    fig.add_trace(go.Scatter(
        x=[dist_left_to_curr_wp[-1]],
        y=[computed_speeds[-1]],
        mode='markers',
        marker=dict(color='red', size=8),
        name="End Speed"
    ))

    # Layout settings
    fig.update_layout(
        title="Deceleration Profile",
        xaxis_title="Distance to Goal (m)",
        yaxis_title="Desired Speed (m/s)",
        legend=dict(x=0, y=1),
        template="plotly_dark",
        width=800,
        height=500
    )

    fig.show()
    return   




class MockConfig:
    """Mock configuration class for testing"""
    def __init__(self):
        self.deceleration_threshold = type('', (), {})()  # Simulate a nested config object
        self.deceleration_threshold.wp2 = 5.0  # Threshold for deceleration
        self.look_ahead = 1
        self.max_y_ep = 1.0  # Maximum allowable cross-track error before full penalty applies

        self.yep_threshold = 0.3  # allowed deviation from line before penalty is applied
        self.lambda_along = 5
        self.lambda_cross = 0.5

@pytest.fixture
def guidance():
    """Fixture to create a Guidance object with test waypoints"""
    wp_list = [
        Waypoint(pose=np.array([0, 0]), speed=2.0),   # Start waypoint
        Waypoint(pose=np.array([10, 0]), speed=0.03)   # Target waypoint
    ]
    return Guidance(dt=0.1, waypoint_list=wp_list, cfg=MockConfig())

def test_speed_laws():
    """ Test how the exponential deceleration function behaves over different distances using Plotly. """
    v_max = 2
    v_intermediate = 1
    v_final = 0.03
    lambda_tot_exp = 8
    deceleration_distance = 11
    
    # Create test range for distance_to_goal (0 to deceleration_distance)
    distances = np.linspace(0, deceleration_distance, 100)
    
    # Exponential speed law
    # speeds = v_final + (v_max - v_final) * np.exp(-lambda_tot_exp * ((deceleration_distance - distances) / deceleration_distance) ** 2)

    # Cubic speed law
    # speeds = v_final + (v_max - v_final)*((distances) / deceleration_distance)**4


    # Sigmoid function for smooth speed transition
    # d_mid = deceleration_distance *0.8
    # speeds = v_final + (v_max - v_final) / (1 + np.exp(lambda_tot_exp * (deceleration_distance - distances - d_mid)))

    # Hybrid exponential/cubic and sigmoid
    transition_threshold = 4  # Use sigmoid only in the last 2 meters before the goal
    speeds = []

    for _, d in enumerate(distances):    
        if d > transition_threshold:
            # Exponential or cubic function for most of the deceleration phase
            speed = v_intermediate + (v_max - v_intermediate) * (d/deceleration_distance)**5 #np.exp(-lambda_tot_exp * ((deceleration_distance - d) / deceleration_distance) ** 2)
        else:
            # Sigmoid smoothing in the last few meters (prevents abrupt stopping)
            d_mid = transition_threshold / 2  # Midpoint for sigmoid transition
            speed = v_final + (v_intermediate - v_final) / (1 + np.exp(-lambda_tot_exp * (d - d_mid)))
        speeds.append(speed)

    # Create interactive plot
    fig = go.Figure()

    # Add speed curve
    fig.add_trace(go.Scatter(
        x=distances,
        y=speeds,
        mode='lines',
        name=f'λ = {lambda_tot_exp}',
        line=dict(color='blue')
    ))

    # Mark start and end speeds
    fig.add_trace(go.Scatter(
        x=[0, deceleration_distance],
        y=[v_final, v_max],
        mode='markers',
        marker=dict(color='red', size=8),
        name="Start/End Speeds"
    ))

    # Layout settings
    fig.update_layout(
        title="Exponential Deceleration Profile",
        xaxis_title="Distance to Goal (m)",
        yaxis_title="Desired Speed (m/s)",
        legend=dict(x=0, y=1),
        template="plotly_dark",
        width=800,
        height=500
    )

    fig.show()


def test_exp_toward_wp1(guidance):
    """Visualize how speed varies with x_ep (along track error) and y_ep (cross track error)"""
    # Create test ranges
    x_ep_values = np.linspace(0, 10, 100)
    y_ep_values = np.linspace(-4, 4, 100) 

    # Create a meshgrid of x_ep and y_ep values
    X, Y = np.meshgrid(x_ep_values, y_ep_values)
    Z = np.zeros_like(X)  # To store speed values for each (x_ep, y_ep)

    # Compute speed values over the grid
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            guidance.x_ep = X[i, j]
            guidance.y_ep = Y[i, j]
            # Calculate the speed at each (x_ep, y_ep) using the provided speed law
            Z[i, j] = guidance.speed_law_cross_and_along_exp()

    # --- Interactive 3D Surface Plot ---
    fig_3d = go.Figure()

    # Add surface plot with the speed data
    fig_3d.add_trace(go.Surface(
        x=X, y=Y, z=Z,
        colorscale='viridis',  # purpor / purpor_r
        contours=dict(
            z=dict(show=True, start=np.min(Z), end=np.max(Z), size=(np.max(Z) - np.min(Z)) / 10, color="black")  # Level curves
        )
    ))

    # Update the layout with axis labels and title
    fig_3d.update_layout(
        title="Speed Surface (x_ep vs y_ep)",
        scene=dict(
            xaxis_title="Along-Track Error (x_ep)",
            yaxis_title="Cross-Track Error (y_ep)",
            zaxis_title="Speed"
        )
    )

    # Show the interactive plot
    fig_3d.show()

    # Optionally save the figure as a PDF (adjust the path as needed)
    save_fig_plotly(output_dir='../masteroppgave-rapport/figs/', fig_name='pid-exp-speed-law-xep-yep', file_format='pdf', fig=fig_3d)

    # Ensure the test runs successfully
    assert True


def test_adaptive_speed_nominal(guidance):
    """Test that the function returns reasonable speeds"""
    guidance.x_ep = 0.0  # Halfway to the waypoint
    guidance.y_ep = 0.0  # No cross-track error

    speed = guidance.adaptive_speed_law()
    assert 1.0 <= speed <= 2.0, "Speed should be within a valid range"

def test_adaptive_speed_cross_track_penalty(guidance):
    """Test that cross-track error reduces speed"""
    guidance.x_ep = 8.0  # Halfway to the waypoint
    guidance.y_ep = 1.0  # Some cross-track error

    speed = guidance.adaptive_speed_law()
    expected_factor = 1 - 0.5 * (abs(guidance.y_ep) / guidance.cfg.max_y_ep)
    assert speed <= 2.0 * expected_factor, "Speed should be reduced due to cross-track error"

def test_adaptive_speed_minimum_limit(guidance):
    """Test that speed never drops below 0.1"""
    guidance.x_ep = 10.0  # At the waypoint
    guidance.y_ep = 0.0  # Max cross-track error

    speed = guidance.adaptive_speed_law()
    a = 1
    assert speed >= 0.1, "Speed should not drop below 0.1"


def test_visualize_speed_law(guidance):
    """Visualize how speed varies with x_ep (along track error) and y_ep (cross track error)"""
    # Create test ranges
    x_ep_values = np.linspace(0, 10, 100)  # Along-track error from 0 to 10
    y_ep_values = np.linspace(-2, 2, 100)  # Cross-track error from -2 to 2

    X, Y = np.meshgrid(x_ep_values, y_ep_values)
    Z = np.zeros_like(X)

    # Compute speed values over the grid
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            guidance.x_ep = X[i, j]
            guidance.y_ep = Y[i, j]
            Z[i, j] = guidance.adaptive_speed_law_gauss_exponential()

    # --- Interactive 3D Surface Plot ---
    fig_3d = go.Figure()
    fig_3d.add_trace(go.Surface(x=X, y=Y, z=Z, colorscale='Viridis'))

    fig_3d.update_layout(
        title="Speed Surface (x_ep vs y_ep)",
        scene=dict(
            xaxis_title="Along-Track Error (x_ep)",
            yaxis_title="Cross-Track Error (y_ep)",
            zaxis_title="Speed"
        )
    )
    fig_3d.show()
    save_fig_plotly(output_dir='../masteroppgave-rapport/figs/', fig_name='pid-speed-law-xep', file_format='pdf', fig=fig_3d)

    # Ensure test runs successfully
    assert True