import time
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from .common_utils import FrameRateCounter, DataAligner, AlignedDataCollection, PointCloudUtils
from cv_bridge import CvBridge
import cv2
from sensor_msgs_py import point_cloud2
from .grasp_model_client import GraspModelClient
import numpy as np
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
import threading
from .hdz_user_interface import HdzUserInterface


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

        # self.pcl_np = None
        # self.pcl_mask = None
        # self.pcl_np_frame_name = ""
        # self.user_img_mask = np.zeros((480, 640), dtype=bool)
        # self.user_img_mask[200:400, 300:500] = True

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

        self.grasp_model_client = GraspModelClient("localhost", 50051)

        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.user_interface = HdzUserInterface(
            infer_callback=self.__infer_callback,
            grasp_callback=self.__grasp_callback,
            logger=get_logger("hdz_user_interface"),
        )

    def info_callback(self, msg):
        self.pcl_util.set_intrinsics(msg.k)

    def rgb_callback(self, msg: Image):
        self.data_aligner.add_data("rgb", msg)

    def depth_callback(self, msg: Image):
        self.data_aligner.add_data("depth", msg)

    def align_callback(self, data_collection: AlignedDataCollection):
        # self.data_aligner_fps_counter.tick()
        # self.data_aligner_fps_counter.print_info("Data aligner: ")

        rgb_msg: Image = data_collection.data_dict["rgb"].data
        depth_msg: Image = data_collection.data_dict["depth"].data

        rgb_np = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_np = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        self.user_interface.update_rgb_depth(rgb_np, depth_np, rgb_msg.header.frame_id, rgb_msg.header.stamp)
        # # Normalize depth image for display
        # depth_image_normalized = cv2.normalize(depth_np, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # # Apply colormap to depth image for better visualization
        # depth_image_colored = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

        # # Mix the RGB and depth images
        # mixed_image = cv2.addWeighted(rgb_np, 0.5, depth_image_colored, 0.5, 0)
        # mask_image = cv2.cvtColor(self.user_img_mask.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
        # mixed_image = cv2.addWeighted(mixed_image, 0.5, mask_image, 0.5, 0)
        # cv2.imshow("Mixed Image", mixed_image)
        # # cv2.imshow("RGB Image", rgb_np)
        # # cv2.imshow("Depth Image", depth_image_colored)
        # cv2.waitKey(1)

        # rgb_timestamp = self.data_aligner.get_timestamp(rgb_msg)
        # depth_timestamp = self.data_aligner.get_timestamp(depth_msg)
        # if abs(rgb_timestamp - depth_timestamp) > 0.01:
        #     self.get_logger().warn(
        #         f"Timestamp mismatch between RGB ({rgb_timestamp}) and depth images ({depth_timestamp}). Diff = {rgb_timestamp - depth_timestamp}"
        #     )

        # depth_np = self.pcl_util.convert_depth_msg_to_np(depth_msg)

        # try:
        #     pcl_np, pcl_mask = self.pcl_util.generate_pcl_np(depth_np, self.user_img_mask)
        # except ValueError:
        #     return

        # self.pcl_np = pcl_np
        # self.pcl_mask = pcl_mask
        # self.pcl_np_frame_name = depth_msg.header.frame_id
        # pcl_msg = self.pcl_util.convert_pcl_to_msg(pcl_np, rgb_msg.header)
        # self.pcl_pub.publish(pcl_msg)

    # def timer_callback(self):
    #     if self.pcl_np is not None:
    #         try:
    #             response = self.grasp_model_client.generate_from_pointcloud(
    #                 pcd=self.pcl_np,
    #                 frame_name=self.pcl_np_frame_name,
    #                 user_mask=self.pcl_mask,
    #             )
    #             self.get_logger().info(str(response))
    #             self.broadcast_grasp_target(response.pose.position, response.pose.orientation, self.pcl_np_frame_name)
    #         except Exception as e:
    #             self.get_logger().error(f"Error in timer_callback: {e}")

    def broadcast_grasp_target(self, position, quat, frame_id, target_name="grasp_target"):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = target_name
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        t.transform.rotation.x = quat.x
        t.transform.rotation.y = quat.y
        t.transform.rotation.z = quat.z
        t.transform.rotation.w = quat.w

        self.tf_broadcaster.sendTransform(t)

    def __infer_callback(self, **kwargs):
        self.get_logger().info(f"Infer callback")
        mask: np.ndarray = kwargs["mask"]
        # rgb: np.ndarray = kwargs["rgb"]
        depth: np.ndarray = kwargs["depth"]
        frame_name: str = kwargs["frame_name"]

        if mask.any():
            try:
                pcl_np, pcl_mask = self.pcl_util.generate_pcl_np(depth, mask)
                response = self.grasp_model_client.generate_from_pointcloud(
                    pcd=pcl_np,
                    frame_name=frame_name,
                    user_mask=pcl_mask,
                )
                self.get_logger().info(str(response))
                self.broadcast_grasp_target(response.pose.position, response.pose.orientation, frame_name)
                return response
            except Exception as e:
                self.get_logger().error(f"Error in timer_callback: {e}")

    def __grasp_callback(self, pose):
        self.get_logger().info(f"Grasp callback. pose = {pose}")


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
