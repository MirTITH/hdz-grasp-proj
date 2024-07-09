import grpc
from grasp_model_grpc_msg import grasp_model_grpc_msg_pb2, grasp_model_grpc_msg_pb2_grpc
from grasp_model_grpc_msg.grasp_model_grpc_msg_pb2 import PointCloud, StrMsg, PoseStamped
import numpy as np
from tqdm import tqdm
import logging

from grasp_model_grpc_msg.utils import PointCloud2Numpy, Numpy2PointCloud


class GraspModelClient:
    def __init__(self, host: str = "localhost", port: int = 50051):
        self.channel = grpc.insecure_channel(f"{host}:{port}")
        self.grasp_stub = grasp_model_grpc_msg_pb2_grpc.GraspModelStub(self.channel)
        self.greeter_stub = grasp_model_grpc_msg_pb2_grpc.GreeterStub(self.channel)

    def generate_from_pointcloud(self, pcd: np.ndarray, frame_name: str = "") -> PoseStamped:
        pcd_msg = Numpy2PointCloud(pcd, frame_name)
        response: PoseStamped = self.grasp_stub.GenerateFromPointCloud(pcd_msg)
        return response

    def __del__(self):
        self.channel.close()


def main():
    grasp_model_client = GraspModelClient("localhost", 50051)
    for i in tqdm(range(100)):
        pcd = np.random.rand(640 * 480, 3).astype(np.float32)
        response = grasp_model_client.generate_from_pointcloud(pcd)
        print(response)


if __name__ == "__main__":
    logging.basicConfig()
    main()
