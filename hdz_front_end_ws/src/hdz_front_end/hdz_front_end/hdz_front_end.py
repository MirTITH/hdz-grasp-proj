import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from .fps_counter import FrameRateCounter
from cv_bridge import CvBridge
import cv2

# from dataclasses import dataclass


def ShowImage(img_msg: Image, window_name: str):
    cv_image = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    cv2.imshow(window_name, cv_image)
    cv2.waitKey(1)


class DataAligner:
    def __init__(self, data_names: set, align_to: str, callback, timestamp_error=0.01):
        super().__init__()

        self.data_dict = dict()
        for name in data_names:
            self.data_dict[name] = {
                "timestamp": -1.0,
                "data": None,
            }

        self.align_to = align_to
        self.callback = callback  # function to call when all data are available
        self.called_callback = False
        self.timestamp_error = timestamp_error

    def __get_timestamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def is_aligned(self):
        reference_timestamp = self.data_dict[self.align_to]["timestamp"]

        for _, data_with_time in self.data_dict.items():
            if data_with_time["data"] is None:
                return False

            if abs(data_with_time["timestamp"] - reference_timestamp) > self.timestamp_error:
                return False

        return True

    def add_data(self, name, data, timestamp=None):
        if name not in self.data_dict:
            raise ValueError(f"Data name '{name}' not in the list of data names: {self.data_dict.keys()}")

        if timestamp is None:
            timestamp = self.__get_timestamp(data)

        if name == self.align_to:
            self.called_callback = False

        self.data_dict[name] = {
            "timestamp": timestamp,
            "data": data,
        }

        if (not self.called_callback) and self.is_aligned():
            self.called_callback = True
            self.callback(self.data_dict)


class HdzFrontEndNode(Node):
    def __init__(self):
        super().__init__("hdz_front_end")

        self.rgb_subscriber = self.create_subscription(Image, "/camera/camera/color/image_raw", self.rgb_callback, 10)
        self.rgb_fps_counter = FrameRateCounter(1000)

        self.pcl_subscriber = self.create_subscription(
            PointCloud2, "/camera/camera/depth/color/points", self.pcl_callback, 10
        )
        self.pcl_fps_counter = FrameRateCounter(1000)

        self.data_aligner = DataAligner({"rgb", "pcl"}, align_to="pcl", callback=self.align_callback)
        self.data_aligner_fps_counter = FrameRateCounter(1000)

    def rgb_callback(self, msg: Image):
        self.data_aligner.add_data("rgb", msg)

        self.rgb_fps_counter.tick()
        # fps = self.rgb_fps_counter.get_fps(2)
        # avg_fps = self.rgb_fps_counter.get_fps()
        # self.get_logger().info(
        #     f"Received img: size: {msg.width}x{msg.height}, FPS: {fps:.2f}, Avg FPS: {avg_fps:.2f}, timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        # )

        # ShowImage(msg, "RGB Image")

    def pcl_callback(self, msg: PointCloud2):
        self.data_aligner.add_data("pcl", msg)

        self.pcl_fps_counter.tick()
        # fps = self.pcl_fps_counter.get_fps(2)
        # avg_fps = self.pcl_fps_counter.get_fps()
        # self.get_logger().info(
        #     f"Received PCL: FPS: {fps:.2f}, Avg FPS: {avg_fps:.2f}, timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        # )

    def align_callback(self, data_dict: dict):
        self.data_aligner_fps_counter.tick()
        fps = self.data_aligner_fps_counter.get_fps(2)
        avg_fps = self.data_aligner_fps_counter.get_fps()
        rgb_msg = data_dict["rgb"]["data"]
        pcl_msg = data_dict["pcl"]["data"]

        rgb_timestamp = data_dict["rgb"]["timestamp"]
        pcl_timestamp = data_dict["pcl"]["timestamp"]

        self.get_logger().info(f"Aligned data: FPS: {fps:.2f}, Avg FPS: {avg_fps:.2f}, RGB timestamp: {rgb_timestamp}, PCL timestamp: {pcl_timestamp}")

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
