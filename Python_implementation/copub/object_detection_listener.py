import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ObjectDistanceListener(Node):
    def __init__(self):
        super().__init__('object_detection_listener')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'copub',
            self.listener_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        realX = msg.data[0]
        realY = msg.data[1]

        self.get_logger().info(f"Received object distance:")
        self.get_logger().info(f"  - RealX: {realX}")
        self.get_logger().info(f"  - RealY: {realY}")

        print("Received object distance:")
        print(f"  - RealX: {realX}")
        print(f"  - RealY: {realY}")

def main(args=None):
    rclpy.init(args=args)
    object_distance_listener = ObjectDistanceListener()
    rclpy.spin(object_distance_listener)
    object_distance_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
