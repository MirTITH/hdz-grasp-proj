from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from typing import Optional
from std_msgs.msg import Header


class PointCloudUtils:
    def __init__(self, intrinsics=None):
        super().__init__()
        self.bridge = CvBridge()
        self.intrinsics = intrinsics

    def set_intrinsics(self, intrinsics):
        self.intrinsics = intrinsics

    def convert_depth_msg_to_np(self, depth_msg: Image):
        return self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")

    def generate_pcl_np(
        self,
        depth_image: np.ndarray,
        additional_mask: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        if self.intrinsics is None:
            return None

        fx, fy, cx, cy = self.intrinsics[0], self.intrinsics[4], self.intrinsics[2], self.intrinsics[5]
        height, width = depth_image.shape
        points = []

        # Generate a grid of coordinates corresponding to the pixel locations
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Mask out points where depth is zero
        mask = depth_image > 0

        if additional_mask is not None:
            mask = np.logical_and(additional_mask, mask)

        z = depth_image[mask] / 1000.0  # Convert depth image from mm to meters
        u = u[mask]
        v = v[mask]

        # Calculate x, y, z coordinates
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Stack x, y, z into a single array of shape (N, 3)
        points = np.vstack((x, y, z)).transpose()

        return points

    def convert_pcl_to_msg(
        self,
        points: np.ndarray,
        header: Header,
    ) -> PointCloud2:

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud = pc2.create_cloud(header, fields, points)
        return point_cloud
