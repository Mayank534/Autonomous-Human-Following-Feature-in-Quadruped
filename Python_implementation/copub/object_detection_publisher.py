import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
from .realsense_camera import RealsenseCamera
from .mask_rcnn import MaskRCNN
import numpy as np
import os

cnt = 0
# Ensure the correct Qt platform plugin is used
os.environ["QT_QPA_PLATFORM"] = "xcb"
# Initialize camera and MaskRCNN
rs = RealsenseCamera()
mrcnn = MaskRCNN()

# Function to resize the frames to fit within the screen
def resize_frame(frame, max_height=600):
    height, width = frame.shape[:2]
    if height > max_height:
        scale_factor = max_height / height
        new_width = int(width * scale_factor)
        return cv2.resize(frame, (new_width, max_height))
    return frame

def custom_round(depth_cm):
    if depth_cm is None:
        return None

    # Convert depth to millimeters
    depth_mm = depth_cm * 100

    # Determine the rounded value
    rounded_value = int(depth_mm)
    if depth_mm - rounded_value >= 0.5:
        rounded_value += 1

    # Convert back to centimeters
    rounded_depth_cm = rounded_value / 100

    return rounded_depth_cm

def measure_object_distance(depth_frame, centers):
    if not centers:
        return None, None, None  # Return None for all values if no objects are detected

    # Assuming the first detected person
    cx, cy = centers[0]
    depth_mm = depth_frame[cy, cx]
    depth_m = depth_mm / 1000.0  # Convert depth to meters
    image_height, image_width = depth_frame.shape

    # Calculate the angle θ between the center of the image and the point (cx, cy)
    theta = np.arctan2(cy - image_height / 2, cx - image_width / 2)

    # Calculate real-world coordinates
    realX = depth_m * np.sin(theta)
    realY = depth_m * np.cos(theta)

    return depth_m, realX, realY

class ObjectDistancePublisher(Node):
    def __init__(self):
        super().__init__('object_detection_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'copub', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.realX = 0.0
        self.realY = 0.0

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.realX, self.realY]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: realX={self.realX}, realY={self.realY}')
        print(f'Published: realX={self.realX}, realY={self.realY}')  # Additional debug print\
        # rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDistancePublisher()  # Create the publisher node instance

    published = False  # Flag to track if values are published
    
    while True:
        # Get frame in real time from Realsense camera
        ret, bgr_frame, depth_frame = rs.get_frame_stream()

        if not ret:
            print("Failed to get frames from Realsense camera.")
            continue

        # Resize frames to fit within the screen
        bgr_frame_resized = resize_frame(bgr_frame)
        depth_frame_resized = resize_frame(depth_frame)

        # Get object mask
        boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame_resized)

        # Draw object mask
        bgr_frame_resized = mrcnn.draw_object_mask(bgr_frame_resized)

        # Show depth info of the objects
        mrcnn.draw_object_info(bgr_frame_resized, depth_frame_resized)

        # Measure the distance to the first detected person
        depth_m, realX, realY = measure_object_distance(depth_frame_resized, centers)

        if depth_m is not None and not published:  # Check if there is a detection and not yet published
            node.realX = realX
            node.realY = realY
            node.timer_callback()  # Publish the values once
            # published = True  # Set published flag to True

        # Print the values
        print("Depth (m):", depth_m)
        print("realX:", realX)
        print("realY:", realY)

        # Display frames
        cv2.imshow("Depth Frame", depth_frame_resized)
        cv2.imshow("BGR Frame", bgr_frame_resized)

        key = cv2.waitKey(1)
        if key == 27:
            break

    rs.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
