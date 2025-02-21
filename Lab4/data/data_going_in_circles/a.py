import csv
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3

# Path to the ROS 2 bag file
db3_file_path = '/home/jaiaditya/Lab4/data/data_going_in_circles/data_going_in_circles_0.db3'

# Set up the CSV file
csv_file_path = 'imu_data.csv'
csv_columns = [
    'timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
    'mag_field_x', 'mag_field_y', 'mag_field_z',
    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'
]

# Open database connection
conn = sqlite3.connect(db3_file_path)
cursor = conn.cursor()

# Query the database for IMU data messages
cursor.execute("SELECT timestamp, topic_id, data FROM messages")
rows = cursor.fetchall()

# Initialize ROS 2 context
rclpy.init()

with open(csv_file_path, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=csv_columns)
    writer.writeheader()

    for row in rows:
        timestamp, topic, data = row
        
        # Check the message type (assumes IMU data topic is known)
        if topic == '/imu':  # Replace with your IMU topic name
            msg_type = 'sensor_msgs/msg/Imu'  # Replace with actual message type
            imu_msg = deserialize_message(data, get_message(msg_type))

            # Extract IMU message data
            csv_row = {
                'timestamp': timestamp,
                'orientation_x': imu_msg.orientation.x,
                'orientation_y': imu_msg.orientation.y,
                'orientation_z': imu_msg.orientation.z,
                'orientation_w': imu_msg.orientation.w,
                'mag_field_x': imu_msg.magnetic_field.x,
                'mag_field_y': imu_msg.magnetic_field.y,
                'mag_field_z': imu_msg.magnetic_field.z,
                'linear_acceleration_x': imu_msg.linear_acceleration.x,
                'linear_acceleration_y': imu_msg.linear_acceleration.y,
                'linear_acceleration_z': imu_msg.linear_acceleration.z,
                'angular_velocity_x': imu_msg.angular_velocity.x,
                'angular_velocity_y': imu_msg.angular_velocity.y,
                'angular_velocity_z': imu_msg.angular_velocity.z
            }

            # Write row to CSV
            writer.writerow(csv_row)

# Close resources
conn.close()
rclpy.shutdown()

print(f"IMU data successfully exported to {csv_file_path}")
