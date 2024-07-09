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

import numpy as np

# from dataclasses import dataclass


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

        self.pcl_util = PointCloudUtils()

        self.pcl_pub = self.create_publisher(PointCloud2, "hdz_front_end/point_cloud", 10)
        self.pcl_pub_fps_counter = FrameRateCounter(1000)

        # self.pcl_subscriber = self.create_subscription(
        #     PointCloud2, "/camera/camera/depth/color/points", self.pcl_callback, 10
        # )

        self.info_subscription = self.create_subscription(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.info_callback, 10
        )

        self.data_aligner = DataAligner(
            data_configs=[
                {"name": "rgb", "maxlen": 5},
                {"name": "pcl", "maxlen": 5},
            ],
            reference_data="pcl",
            callback=self.align_callback,
            timestamp_tolerance=1.0 / 30.0 / 2.0,
        )
        self.data_aligner_fps_counter = FrameRateCounter(1000)

    def info_callback(self, msg):
        self.pcl_util.set_intrinsics(msg.k)

    def rgb_callback(self, msg: Image):
        self.data_aligner.add_data("rgb", msg)

        # self.rgb_fps_counter.tick()
        # fps = self.rgb_fps_counter.get_fps(2)
        # avg_fps = self.rgb_fps_counter.get_fps()
        # self.get_logger().info(
        #     f"Received img: size: {msg.width}x{msg.height}, FPS: {fps:.2f}, Avg FPS: {avg_fps:.2f}, timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        # )

        # ShowImage(msg, "RGB Image")

    def depth_callback(self, msg: Image):
        self.pcl_pub_fps_counter.tick()

        depth_np = self.pcl_util.convert_depth_msg_to_np(msg)
        pcl_np = self.pcl_util.generate_pcl_np(depth_np)
        pcl_msg = self.pcl_util.convert_pcl_to_msg(pcl_np, msg.header)

        self.pcl_pub_fps_counter.print_info(f"pcl_np.shape: {pcl_np.shape} ")

        if pcl_msg is not None:
            self.pcl_pub.publish(pcl_msg)

    def pcl_callback(self, msg: PointCloud2):
        self.data_aligner.add_data("pcl", msg)

        # self.pcl_fps_counter.tick()
        # fps = self.pcl_fps_counter.get_fps(2)
        # avg_fps = self.pcl_fps_counter.get_fps()
        # self.get_logger().info(
        #     f"Received PCL: FPS: {fps:.2f}, Avg FPS: {avg_fps:.2f}, timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        # )

    def align_callback(self, data_collection: AlignedDataCollection):
        self.data_aligner_fps_counter.tick()
        fps = self.data_aligner_fps_counter.get_fps(2)
        avg_fps = self.data_aligner_fps_counter.get_fps()
        rgb_msg: Image = data_collection.data_dict["rgb"].data
        pcl_msg: PointCloud2 = data_collection.data_dict["pcl"].data
        pcl_numpy = point_cloud2.read_points_numpy(pcl_msg, ["x", "y", "z"])

        rgb_timestamp = data_collection.data_dict["rgb"].timestamp
        pcl_timestamp = data_collection.data_dict["pcl"].timestamp

        # self.get_logger().info(
        #     f"Aligned data: FPS: {fps:.2f}, Avg FPS: {avg_fps:.2f}, RGB timestamp: {rgb_timestamp}, PCL timestamp: {pcl_timestamp}"
        # )

        # self.get_logger().info(f"PCL shape: {pcl_numpy.shape}, rgb_msg shape: {rgb_msg.width}x{rgb_msg.height}")

        # Do something with the aligned data


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
