
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        # Set the initial pose values (e.g., x, y, z, orientation)
        initial_pose_msg.pose.pose.position.x = 0.0
        initial_pose_msg.pose.pose.position.y = 0.0
        initial_pose_msg.pose.pose.position.z = 0.0
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0
        initial_pose_msg.pose.covariance = [0.25      , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.25      , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.06853892]
        
        # Publish the message
        self.publisher_.publish(initial_pose_msg)
        self.get_logger().info("Initial Pose Published")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
