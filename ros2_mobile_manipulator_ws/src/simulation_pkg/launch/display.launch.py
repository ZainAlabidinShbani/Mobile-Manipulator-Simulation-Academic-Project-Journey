"""Alias for rviz_display.launch.py.

Allows both:
  ros2 launch simulation_pkg display.launch.py
  ros2 launch simulation_pkg rviz_display.launch.py
"""
from simulation_pkg.launch.rviz_display import generate_launch_description  # noqa: F401
