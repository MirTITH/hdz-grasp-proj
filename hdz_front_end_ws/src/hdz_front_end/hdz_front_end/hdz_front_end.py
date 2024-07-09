import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from .fps_counter import FrameRateCounter
from cv_bridge import CvBridge
import cv2
from .data_aligner import DataAligner, AlignedDataCollection
from sensor_msgs_py import point_cloud2
from .point_cloud_utils import PointCloudUtils
from .grasp_model_client import GraspModelClient

import numpy as np


def ShowImage(img_msg: Image, window_name: str):
    cv_image = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    cv2.imshow(window_name, cv_image)
    cv2.waitKey(1)


def ShowDepth(img_msg: Image, window_name: str):
    # Convert ROS Image message to OpenCV image
    depth_image = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding="16UC1")

    # Normalize depth image for display
    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Apply colormap to depth image for better visualization
    depth_image_colored = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

    # Display the image
    cv2.imshow("Depth Image", depth_image_colored)
    cv2.waitKey(1)


class HdzFrontEndNode(Node):
    def __init__(self):
        super().__init__("hdz_front_end")

        self.rgb_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self.rgb_callback, 10)

        self.depth_sub = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.info_subscription = self.create_subscription(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.info_callback, 10
        )

        self.pcl_util = PointCloudUtils()

        self.pcl_pub = self.create_publisher(PointCloud2, "hdz_front_end/point_cloud", 10)

        self.data_aligner = DataAligner(
            data_configs=[
                {"name": "rgb", "maxlen": 5},
                {"name": "depth", "maxlen": 5},
            ],
            reference_data="rgb",
            callback=self.align_callback,
            timestamp_tolerance=1.0 / 30.0 / 2.0,
        )
        self.data_aligner_fps_counter = FrameRateCounter(1000)

        # self.grasp_model_client = GraspModelClient("localhost", 50051)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def info_callback(self, msg):
        self.pcl_util.set_intrinsics(msg.k)

    def rgb_callback(self, msg: Image):
        self.data_aligner.add_data("rgb", msg)

    def depth_callback(self, msg: Image):
        self.data_aligner.add_data("depth", msg)

    def align_callback(self, data_collection: AlignedDataCollection):
        self.data_aligner_fps_counter.tick()
        self.data_aligner_fps_counter.print_info("Data aligner: ")

        rgb_msg: Image = data_collection.data_dict["rgb"].data
        depth_msg: Image = data_collection.data_dict["depth"].data

        rgb_np = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_np = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        # Normalize depth image for display
        depth_image_normalized = cv2.normalize(depth_np, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # Apply colormap to depth image for better visualization
        depth_image_colored = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

        # Mix the RGB and depth images
        mixed_image = cv2.addWeighted(rgb_np, 0.5, depth_image_colored, 0.5, 0)
        cv2.imshow("Mixed Image", mixed_image)
        cv2.imshow("RGB Image", rgb_np)
        cv2.imshow("Depth Image", depth_image_colored)
        cv2.waitKey(1)

        rgb_timestamp = self.data_aligner.get_timestamp(rgb_msg)
        depth_timestamp = self.data_aligner.get_timestamp(depth_msg)
        if abs(rgb_timestamp - depth_timestamp) > 0.01:
            self.get_logger().warn(
                f"Timestamp mismatch between RGB ({rgb_timestamp}) and depth images ({depth_timestamp}). Diff = {rgb_timestamp - depth_timestamp}"
            )

        depth_np = self.pcl_util.convert_depth_msg_to_np(depth_msg)
        pcl_np = self.pcl_util.generate_pcl_np(depth_np)
        if pcl_np is not None:
            pcl_msg = self.pcl_util.convert_pcl_to_msg(pcl_np, rgb_msg.header)
            self.pcl_pub.publish(pcl_msg)

    def timer_callback(self):
        self.get_logger().info("Timer callback")


def main(args=None):
    rclpy.init(args=args)

    hdz_front_end_node = HdzFrontEndNode()

    rclpy.spin(hdz_front_end_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hdz_front_end_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
