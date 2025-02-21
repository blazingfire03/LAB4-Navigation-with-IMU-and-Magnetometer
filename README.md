#  Navigation with IMU and Magnetometer

## Overview
This lab focuses on building a navigation stack using GPS and IMU sensors. The objective is to explore the strengths and limitations of these sensors and introduce sensor fusion techniques. The main tasks include:
- Data collection
- Magnetometer calibration
- Yaw estimation
- Forward velocity estimation
- Dead reckoning with the IMU

## Hardware and Software Requirements

### Hardware
- **GNSS Puck**: USB-based GPS sensor
- **VectorNav VN-100 IMU**: USB-based inertial measurement unit

### Software
- **ROS2**: Robot Operating System for sensor integration and data collection
- **GPS Driver**: Obtained from Lab 1
- **IMU Driver**: Obtained from Lab 3
- **MATLAB or Python**: Used for data analysis and visualization

## Data Collection Steps

### 1. Setup and Preparation
- Mount the IMU in the center of the car’s dashboard with the x-axis pointing forward.
- Fix the GPS puck securely on the car roof.
- Connect both sensors to a single laptop for data logging.

### 2. Launch and Verify Sensors
- Create a launch file to start both GPS and IMU driver nodes.
- Verify sensor data publishing using:
  ```bash
  ros2 topic echo /gps
  ros2 topic echo /imu
  ```

### 3. Collect Datasets
- **Car Donuts (Circular Driving):**
  - Start recording a rosbag named `data_going_in_circles`.
  - Drive in circles for 4-5 full loops.
  - Stop recording but keep the sensors running.
- **Mini Boston Tour:**
  - Start recording a rosbag named `data_driving`.
  - Drive around Boston for at least 2-3 kilometers with at least 10 turns.
  - Stop recording once you return to the starting point.

## Data Analysis Steps

### 1. Magnetometer Calibration
- Correct hard-iron and soft-iron effects using circular driving data.
- Plot magnetometer readings before and after calibration to validate improvements.

### 2. Yaw Estimation
- Compute yaw from calibrated magnetometer data.
- Integrate gyroscope data to estimate yaw.
- Implement a complementary filter to combine magnetometer and gyroscope yaw estimates.
- Plot and compare different yaw estimation methods.

### 3. Forward Velocity Estimation
- Integrate forward acceleration from the IMU to estimate velocity.
- Compare this estimation with GPS-derived velocity.
- Adjust acceleration to refine the velocity estimate.

### 4. Dead Reckoning
- Integrate forward velocity to compute displacement.
- Use magnetometer heading to transform velocity into easting and northing components.
- Integrate these components to estimate the full trajectory.
- Compare the IMU-derived trajectory with GPS ground truth data.

## Submitting Lab 4

### Repository Structure
- **`LAB4/`** (Main directory)
  - **`src/`**: Contains driver scripts.
  - **`data/`**: Stores collected rosbag files.
  - **`analysis/`**: Includes data analysis scripts and the final report.

### Submission Instructions
1. Push the repository to GitLab.
2. Upload the final report to Canvas as required.

## Code Execution Instructions

### 1. Clone the Repository
```bash
 git clone <repo_url>
 cd LAB4
```

### 2. Ensure ROS2 is Installed and Sourced
Before running any ROS2 commands, source the ROS2 environment:
```bash
source /opt/ros/<ros_distribution>/setup.bash
```

### 3. Launch GPS and IMU Nodes
Use the provided launch file to start both sensor drivers:
```bash
ros2 launch <your_launch_file>.launch.py
```

### 4. Run Data Analysis
Navigate to the `analysis` directory and execute MATLAB or Python scripts to process the collected data.

## Notes
- Ensure all sensor timestamps use system time.
- Continuously monitor sensors during data collection to prevent data loss.
- Adhere to local traffic laws while collecting data in real-world environments.

---

This README provides all necessary details for executing Lab 4, from setup and data collection to analysis and submission. If you encounter any issues, please check the troubleshooting section or reach out to the course instructors.
