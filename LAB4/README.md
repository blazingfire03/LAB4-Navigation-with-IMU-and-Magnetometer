# Lab 4: Navigation with IMU and Magnetometer

## Overview
This lab involves building a navigation stack using GPS and IMU sensors to understand their strengths and drawbacks and introduce sensor fusion techniques. The tasks include data collection, magnetometer calibration, yaw estimation, forward velocity estimation, and dead reckoning with the IMU.

## Hardware and Software Requirements
- **Hardware:**
  - GNSS puck (USB-based GPS sensor)
  - VectorNav VN-100 IMU (USB-based)

- **Software:**
  - ROS2
  - GPS Driver from Lab 1
  - IMU Driver from Lab 3
  - MATLAB or Python for data analysis

## Data Collection Steps
1. **Setup and Preparation:**
   - Mount the IMU in the center of the car's dashboard with the x-axis pointing forward.
   - Fix the GPS puck to the car roof.
   - Connect both sensors to a single laptop.

2. **Launch and Verify Sensors:**
   - Create a launch file to start both GPS and IMU driver nodes.
   - Verify data publishing using `ros2 topic echo gps` and `ros2 topic echo imu`.

3. **Collect Datasets:**
   - **Car Donuts:**
     - Begin a rosbag named `data_going_in_circles`.
     - Drive in circles 4-5 times.
     - Stop recording without turning off the sensors.
   - **Mini Boston Tour:**
     - Begin a rosbag named `data_driving`.
     - Drive around Boston for at least 2-3 kilometers with a minimum of 10 turns.
     - Stop recording once you return to the starting point.

## Data Analysis Steps
1. **Magnetometer Calibration:**
   - Correct for hard-iron and soft-iron effects using the circular driving data.
   - Plot and compare magnetometer data before and after calibration.

2. **Yaw Estimation:**
   - Calculate yaw from calibrated magnetometer data.
   - Integrate gyro data to get yaw angle.
   - Use a complementary filter to combine magnetometer and gyro yaw estimates.
   - Compare and plot the results.

3. **Forward Velocity Estimation:**
   - Integrate forward acceleration to estimate velocity.
   - Compare with GPS-derived velocity.
   - Adjust the forward acceleration to correct the velocity estimate.

4. **Dead Reckoning:**
   - Integrate forward velocity to obtain displacement.
   - Use magnetometer heading to rotate velocity into easting and northing components.
   - Integrate these components to estimate the trajectory.
   - Compare the IMU-based trajectory with GPS data.

## Submitting Lab 4
1. **Repo Structure:**
   - Create a directory `LAB4` in your class repo `EECE5554`.
   - Sub-directories:
     - `src`: Contains driver files.
     - `data`: Contains rosbag files.
     - `analysis`: Contains analysis files and report.

2. **Submission:**
   - Push the repo to GitLab.
   - Upload the report to Canvas.

## Code Execution Instructions
1. Clone the repository and navigate to the `LAB4` directory.
2. Ensure ROS2 is installed and sourced.
3. To launch both GPS and IMU nodes, use the provided launch file:
   ```bash
   ros2 launch <your_launch_file>.launch.py
   ```
4. For data analysis, run the MATLAB or Python scripts located in the `analysis` directory.

## Notes
- Ensure all sensor timestamps use system time.
- Monitor the sensors during data collection to avoid interruptions.
- Follow all local traffic laws during data collection.