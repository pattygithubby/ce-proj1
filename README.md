# TurtleBot3 Obstacle Avoidance & Victim Detection System

This repository contains a ROS 2 node for autonomous navigation, obstacle avoidance, and victim detection using a TurtleBot3 robot. The system uses LiDAR and RGB sensors to make real-time movement decisions, optimize speed, and detect red-colored victims in a simulated disaster environment.

## Features

- **Dynamic Obstacle Avoidance**  
  Divides the 360° LiDAR scan into sectors and computes directional scores to navigate safely around obstacles.
  
- **Victim Detection via RGB Sensor**  
  Uses the ISL29125 RGB sensor to identify red-colored objects on the ground as potential victims. When a victim is detected, an onboard LED is activated.

- **Speed Optimization**  
  Linear and angular speeds are dynamically adjusted based on proximity to obstacles using linear interpolation.

- **Collision Logging**  
  Tracks and logs the number of collisions and their timing to evaluate navigation safety.

- **Metric Collection**  
  Collects metrics such as speed, collisions, and victims found, exporting the data to an Excel file at the end of the run.

- **Modular ROS 2 Node**  
  Built using ROS 2 and Python, with clean separation between logic components and sensor processing.

## Project Structure

- `turtlebot3_obstacle_detection.py`:  
  Main ROS 2 node that implements all navigation, detection, and logging functionality.

## Dependencies

- ROS 2 (tested with Humble or Foxy)
- Python 3.8+
- `rclpy`, `geometry_msgs`, `sensor_msgs`
- `gpiozero` (for LED)
- `smbus2` or `smbus` (for I2C RGB sensor)
- `numpy`, `pandas` (for data processing and export)
