from typing import Dict, Any
import numpy as np
from generate_results.types import Metric, MetricCollection
import plotly.graph_objects as go


def expand_metric_collection(metric_list: list[Metric], metric_collection: MetricCollection) -> MetricCollection:
    for i, metric in enumerate(metric_list):
        if metric.metric_type not in metric_collection.metric_collection:
            metric_collection.metric_collection[metric.metric_type] = []

        metric_collection.metric_collection[metric.metric_type].append(metric.value)
    return metric_collection


def plot_box(ax, center_north, center_east, yaw_rad, length, width, legend_label=None):
    """
    Plots a rectangular box (representing the vessel) given a center, yaw angle, and dimensions.
    :param ax: The axis on which to plot the box.
    :param center_north: North coordinate of the box center.
    :param center_east: East coordinate of the box center.
    :param yaw_deg: Heading (yaw angle) of the box in degrees (clockwise from north).
    :param length: Length of the box.
    :param width: Width of the box.
    :param legend_label: Optional label for the legend.
    """

    # Define box vertices in local coordinates (centered at origin)
    vertices = np.array([
        [-length/2, -width/2],  # Bottom-left
         [length/2, -width/2],  # Bottom-right
         [length/2, width/2],   # Top-right
         [-length/2, width/2]   # Top-left
    ])

    # Rotation matrix for yaw angle (90 degrees about Z-axis)
    R_yaw = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad)],
        [np.sin(yaw_rad), np.cos(yaw_rad)]
    ])

    R_z = np.array([
        [0, -1],
        [1, 0]
    ])

    R_x = np.array([
        [1, 0],
        [0, -1]
    ])

    # Apply yaw rotation
    rotated_vertices = (R_yaw @ vertices.T).T

    # Rotate 90 degrees about the z-axis
    rotated_vertices = (R_z @ rotated_vertices.T).T

    # Rotate 180 degrees about the new x-axis
    rotated_vertices = (R_x @ rotated_vertices.T).T

    # Translate the vertices to the given center position
    translated_vertices = rotated_vertices + np.array([center_east, center_north])

    # Close the polygon by repeating the first vertex
    translated_vertices = np.vstack([translated_vertices, translated_vertices[0, :]])

    # Plot the box
    ax.plot(translated_vertices[:, 0], translated_vertices[:, 1], ':',
            color=[0.1, 0.1, 0.1], alpha=0.6, linewidth=1, label=legend_label)
    

def plot_box_plotly(fig, centers_north, centers_east,
                    yaws_rad, length, width, name=None):
    """
    Plots a rectangular box (representing the vessel) using Plotly.
    :param fig: The Plotly figure object to which the box will be added.
    :param center_north: North coordinate of the box center.
    :param center_east: East coordinate of the box center.
    :param yaw_rad: Heading (yaw angle) of the box in radians.
    :param length: Length of the box.
    :param width: Width of the box.
    :param name: Optional label for the legend.
    """
    all_x, all_y = [], []

    R_z = np.array([
        [0, -1],
        [1, 0]
    ])

    R_x = np.array([
        [1, 0],
        [0, -1]
    ])

    if np.ndim(centers_north) == 0:
        # Define box vertices in local coordinates (centered at origin)
        vertices = np.array([
            [-length / 2, -width / 2],  # Bottom-left
            [length / 2, -width / 2],   # Bottom-right
            [length / 2, width / 2],    # Top-right
            [-length / 2, width / 2],   # Top-left
            [-length / 2, -width / 2]   # Closing the box
        ])
    
        # Rotation matrix for yaw angle
        R_yaw = np.array([
            [np.cos(yaws_rad), -np.sin(yaws_rad)],
            [np.sin(yaws_rad), np.cos(yaws_rad)]
        ])

        # Apply yaw rotation
        rotated_vertices = (R_yaw @ vertices.T).T

        # Rotate 90 degrees about the z-axis
        rotated_vertices = (R_z @ rotated_vertices.T).T

        # Rotate 180 degrees about the new x-axis
        rotated_vertices = (R_x @ rotated_vertices.T).T
        
        # Translate the vertices to the given center position
        translated_vertices = rotated_vertices + np.array([centers_east, centers_north])
        
        # Close the polygon by repeating the first vertex
        translated_vertices = np.vstack([translated_vertices, translated_vertices[0, :]])
        
        # Add the box to the figure
        return go.Scatter(
            x=translated_vertices[:, 0], y=translated_vertices[:, 1], mode='lines',
            line=dict(color='black', dash='dot', width=1),
            name=name if name else None,
            showlegend=name is not None
        )

    
    else:
        # Define box vertices in local coordinates (centered at origin)
        for center_north, center_east, yaw_rad in zip(centers_north, centers_east, yaws_rad):
            # Define box vertices in local coordinates (centered at origin)
            vertices = np.array([
                [-length / 2, -width / 2],  # Bottom-left
                [length / 2, -width / 2],   # Bottom-right
                [length / 2, width / 2],    # Top-right
                [-length / 2, width / 2],   # Top-left
                [-length / 2, -width / 2]   # Closing the box
            ])
        
            # Rotation matrix for yaw angle
            R_yaw = np.array([
                [np.cos(yaw_rad), -np.sin(yaw_rad)],
                [np.sin(yaw_rad), np.cos(yaw_rad)]
            ])

            # Apply yaw rotation
            rotated_vertices = (R_yaw @ vertices.T).T

            # Rotate 90 degrees about the z-axis
            rotated_vertices = (R_z @ rotated_vertices.T).T

            # Rotate 180 degrees about the new x-axis
            rotated_vertices = (R_x @ rotated_vertices.T).T
            
            # Translate the vertices to the given center position
            translated_vertices = rotated_vertices + np.array([center_east, center_north])
            
            # Close the polygon by repeating the first vertex
            translated_vertices = np.vstack([translated_vertices, translated_vertices[0, :]])
            
            # Extract x and y coordinates
            all_x.extend(translated_vertices[:, 0].tolist() + [None])
            all_y.extend(translated_vertices[:, 1].tolist() + [None])
    
        # Add the box to the figure
        return go.Scatter(
            x=all_x, y=all_y, mode='lines',
            line=dict(color='black', dash='dot', width=1),
            name=name if name else None,
            showlegend=name is not None
        )
    
def unwrap_angle(angle_rad: np.ndarray) -> np.ndarray:
    """
    Wraps angles from (-π, π] to [0, 2π).
    
    Parameters:
        angle_rad (np.ndarray or float): Angle(s) in radians within (-π, π].
    
    Returns:
        np.ndarray or float: Wrapped angle(s) within [0, 2π).
    """
    return (angle_rad % (2 * np.pi))  # Ensure the output is in (0, 2π]

def plot_thick_quay(ax, quay_start, quay_end, quay_thickness, quay_normal_vector, color, label=None):
    # Unpack and invert the normal vector
    ny, nx = quay_normal_vector
    offset_x = -nx * quay_thickness
    offset_y = -ny * quay_thickness

    # Create polygon offset in the opposite direction of the normal
    quay_polygon = [
        [quay_start[1], quay_start[0]],
        [quay_end[1], quay_end[0]],
        [quay_end[1] + offset_x, quay_end[0] + offset_y],
        [quay_start[1] + offset_x, quay_start[0] + offset_y],
    ]

    ax.fill(
        [p[0] for p in quay_polygon],
        [p[1] for p in quay_polygon],
        color=color,
        label=label
    )

