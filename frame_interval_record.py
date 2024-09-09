import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import csv
import os

real_sense_color_topic = '/camera/camera/color/image_raw'
orbbec_color_topic = '/camera_eye_in_hand/color/image_raw'


class FrameIntervalRecorder(Node):
    def __init__(self):
        super().__init__('frame_interval_recorder')
        self.subscription = self.create_subscription(
            Image,
            orbbec_color_topic,
            self.listener_callback,
            10)
        self.previous_timestamp = None
        self.csv_file = '/workspace/logging/frame_intervals_ros_optimized_0.csv'

        # Create CSV file and write header
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Frame', 'Age (seconds)'])

        self.frame_count = 0
        self.frame_ages = []

    def listener_callback(self, msg):
        current_timestamp = self.get_clock().now().nanoseconds * 1e-9
        image_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = current_timestamp - image_timestamp
        self.frame_count += 1
        self.frame_ages.append(age)
        #print(len(self.frame_ages))
        if self.frame_count > 1500:
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                for i, age in enumerate(self.frame_ages):
                    writer.writerow([i+1, age])
            rclpy.shutdown()
        if self.frame_count % 100 == 0:
            print(f"Frame {self.frame_count}: {age} seconds")



def main(args=None):
    rclpy.init(args=args)
    frame_interval_recorder = FrameIntervalRecorder()
    rclpy.spin(frame_interval_recorder)
    frame_interval_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
