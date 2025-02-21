import numpy as np
from scipy import signal
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import yaml
from sensor_msgs.msg import Imu, MagneticField, NavSatFix

class IMUGPSAnalyzer:
    def __init__(self):
        """Initialize the analyzer with default parameters"""
        # Initialize ROS2
        rclpy.init()
        
        # Data containers
        self.mag_data = None
        self.gyro_data = None
        self.accel_data = None
        self.gps_data = None
        self.timestamps = None
        
        # Calibration parameters
        self.mag_bias = np.array([0.0, 0.0, 0.0])  # Define based on calibration data
        self.mag_scale = np.array([1.0, 1.0, 1.0])  # Define based on calibration data
        
        # Filter parameters
        self.comp_filter_alpha = 0.98
        self.lpf_cutoff = 0.1
        self.hpf_cutoff = 2.0

    def read_rosbag(self, bag_path):
        """
        Read data from ROS2 bag
        Args:
            bag_path: Path to the rosbag directory
        """
        # Set up reader
        storage_options = StorageOptions(
            uri=str(bag_path),
            storage_id='sqlite3'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Initialize data containers
        imu_data = []
        mag_data = []
        gps_data = []
        
        # Read messages
        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            
            try:
                if topic_name == '/imu':
                    msg = deserialize_message(data, Imu)
                    imu_data.append({
                        'timestamp': timestamp,
                        'angular_velocity': [
                            msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z
                        ],
                        'linear_acceleration': [
                            msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z
                        ]
                    })
                elif topic_name == '/mag':
                    msg = deserialize_message(data, MagneticField)
                    mag_data.append({
                        'timestamp': timestamp,
                        'magnetic_field': [
                            msg.magnetic_field.x,
                            msg.magnetic_field.y,
                            msg.magnetic_field.z
                        ]
                    })
                elif topic_name == '/gps':
                    msg = deserialize_message(data, NavSatFix)
                    gps_data.append({
                        'timestamp': timestamp,
                        'latitude': msg.latitude,
                        'longitude': msg.longitude,
                        'altitude': msg.altitude
                    })
            except Exception as e:
                print(f"Error deserializing data on topic '{topic_name}': {e}")
        
        return imu_data, mag_data, gps_data

    def calibrate_magnetometer(self, raw_mag):
        """
        Calibrate the magnetometer data by applying bias and scale corrections.
        Args:
            raw_mag: np.array of raw magnetometer readings
        Returns:
            np.array of calibrated magnetometer data
        """
        return (raw_mag - self.mag_bias) * self.mag_scale

    # Rest of the class methods...

def main():
    # Initialize analyzer
    analyzer = IMUGPSAnalyzer()
    
    # Read data from rosbags
    print("Reading circles data...")
    circles_data = analyzer.read_rosbag('/home/jaiaditya/Lab4/data/data_going_in_circles/data_going_in_circles_0.db3')
    print("Reading driving data...")
    driving_data = analyzer.read_rosbag('/home/jaiaditya/Lab4/data/data_driving.db3')
    
    # Convert magnetometer data to numpy array
    raw_mag = np.array([msg['magnetic_field'] for msg in circles_data[1]])
    cal_mag = analyzer.calibrate_magnetometer(raw_mag)
    
    # Process driving data
    imu_data, mag_data, gps_data = driving_data
    
    # Extract timestamps and convert to relative time in seconds
    timestamps = np.array([msg['timestamp'] for msg in imu_data])
    timestamps = (timestamps - timestamps[0]) * 1e-9  # Convert nanoseconds to seconds
    dt = np.mean(np.diff(timestamps))
    
    # Calculate yaw angles
    mag_arr = np.array([msg['magnetic_field'] for msg in mag_data])
    mag_yaw = analyzer.calculate_magnetometer_yaw(mag_arr)
    
    gyro_z = np.array([msg['angular_velocity'][2] for msg in imu_data])
    gyro_yaw = analyzer.integrate_gyro_yaw(gyro_z, dt)
    
    # Ensure mag_yaw and gyro_yaw have the same length
    min_len = min(len(mag_yaw), len(gyro_yaw))
    mag_yaw = mag_yaw[:min_len]
    gyro_yaw = gyro_yaw[:min_len]
    
    combined_yaw = analyzer.complementary_filter(mag_yaw, gyro_yaw, dt)
    
    # Process velocity
    accel_data = np.array([msg['linear_acceleration'] for msg in imu_data])[:min_len]
    gps_positions = np.array([[msg['latitude'], msg['longitude']] for msg in gps_data])
    
    # Convert GPS coordinates to local coordinates (assuming small distances)
    ref_lat, ref_lon = gps_positions[0]
    R_earth = 6371000  # Earth radius in meters
    local_x = (gps_positions[:, 1] - ref_lon) * np.cos(np.radians(ref_lat)) * R_earth * np.pi / 180
    local_y = (gps_positions[:, 0] - ref_lat) * R_earth * np.pi / 180
    gps_local = np.column_stack((local_x, local_y))
    
    vel_imu, vel_gps = analyzer.process_velocity(accel_data[:, 0], gps_local, dt)  # Using forward acceleration
    
    # Dead reckoning
    pos_imu_x, pos_imu_y = analyzer.dead_reckoning(vel_imu, combined_yaw, dt)
    
    # Plot results
    analyzer.plot_results(
        raw_mag, cal_mag, mag_yaw, gyro_yaw, combined_yaw,
        vel_imu, vel_gps,
        np.column_stack((pos_imu_x, pos_imu_y)),
        gps_local
    )
    
    # Clean up ROS2
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
