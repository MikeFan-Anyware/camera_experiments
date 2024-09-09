"""3d_cam_latency.py
This script is used to investigate the latency issue known to when using the ORBBEC Gemini 2L
camera with the 3D perception pipeline. The issue is that the camera takes about 1-2 seconds
to take RGB and depth images, which is slow for our purposes. This latency is expected to be
way lower than 1 second. The Realsense camera on Gen 1 system does not have this much latency.

The cause may be:
1. The driver + ROS2 wrapper for the Gemini 2L camera is slow.
2. The camera is slow internally and there is nothing we can do about it.

This script will try to isolate the issue by controling the camera in different ways and measuring
the latency.
"""

import cv2 as cv
import sys
import time
import matplotlib.pyplot as plt
import matplotlib
import csv
import threading


import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import csv
import os

matplotlib.use('TkAgg')

def publish_rgb(node, publisher,data,  width, height, frame_id='', encoding='rgb8', is_bigendian=0):
    """
    Publish the RGB image to a ROS2 topic
    Args:
        node: ROS2 node
        publisher: ROS2 publisher
        data: RGB image data
        width: Width of the image
        height: Height of the image
        timestamp: Timestamp of the image
        frame_id: Frame ID of the image
        encoding: Encoding of the image
        is_bigendian: Is the image bigendian
    """
    # Sub process to publish the RGB image
    # Publish the data
    msg_rgb = Image()
    msg_rgb.header.stamp = node.get_clock().now().to_msg()
    msg_rgb.header.frame_id = frame_id
    msg_rgb.height = height
    msg_rgb.width = width
    msg_rgb.encoding = encoding
    msg_rgb.is_bigendian = is_bigendian
    msg_rgb.data = data
    publisher.publish(msg_rgb)


def ros2_dummy_image_only():
    """
    This function creates a ROS2 node that publishes dummy RGB image data to a topic.
    We eliminates the source of latency from the camera and only measure the latency
    of the ROS2 node.
    """
    # Setup the ROS2 node, reliability policy - best effort
    rclpy.init()
    node = rclpy.create_node('camera_latency_study_node')
    publisher_rgb = node.create_publisher(Image, '/camera_eye_in_hand/color/image_raw', 10)


    # Run countinously
    while rclpy.ok():
        dummy_data = np.random.randint(0, 255, (640, 480, 3), dtype=np.uint8)
        publish_rgb(node, publisher_rgb, dummy_data.tobytes(), 640, 480)
        # Sleep to achieve 30 fps
        time.sleep(1/30)
    rclpy.shutdown()


# Analyze the data
def analyze_data():
    ROS_data = "frame_intervals_ros.csv"
    OpenCV_data = "frame_intervals_opencv.csv"
    PythonSDK_data = "frame_intervals_python_sdk.csv"

    # Read the data
    with open(ROS_data, mode='r') as file:
        reader = csv.reader(file)
        ros_data = list(reader)
        # Find max and average latency
        ros_data = ros_data[1:]
        ros_latency = [float(row[1]) for row in ros_data]
        ros_avg_latency = sum(ros_latency)/len(ros_latency)
        # Plot the latency
        plt.plot(ros_latency)

    with open(OpenCV_data, mode='r') as file:
        reader = csv.reader(file)
        opencv_data = list(reader)
        # Find max and average latency
        opencv_data = opencv_data[1:]
        opencv_latency = [float(row[1]) for row in opencv_data]
        opencv_avg_latency = sum(opencv_latency)/len(opencv_latency)
        # Plot the latency
        plt.plot(opencv_latency)

    with open(PythonSDK_data, mode='r') as file:
        reader = csv.reader(file)
        python_sdk_data = list(reader)
        # Find max and average latency
        python_sdk_data = python_sdk_data[1:]
        python_sdk_latency = [float(row[1]) for row in python_sdk_data]
        python_sdk_avg_latency = sum(python_sdk_latency)/len(python_sdk_latency)
        # Plot the latency
        plt.plot(python_sdk_latency)

    print("ROS2 average latency: ", ros_avg_latency)
    print("OpenCV average latency: ", opencv_avg_latency)
    print("ROS2 max latency: ", max(ros_latency))
    print("OpenCV max latency: ", max(opencv_latency))
    print("Python SDK average latency: ", python_sdk_avg_latency)
    print("Python SDK max latency: ", max(python_sdk_latency))

    plt.legend(["ROS2", "OpenCV", "Python SDK"])
    plt.xlabel("Frame")
    plt.ylabel("Latency (s)")
    plt.title("Latency of Orbbec camera")
    plt.show()


if __name__ == '__main__':
    #eval_capture_open_cv()
    #eval_capture_python_sdk()
    #python_sdk_with_ros2_node()
    ros2_dummy_image_only()
    #analyze_data()
