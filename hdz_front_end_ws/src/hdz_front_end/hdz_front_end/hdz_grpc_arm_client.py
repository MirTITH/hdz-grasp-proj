import grpc
from .hdz_grpc_msg import hdz_grpc_msg_pb2, hdz_grpc_msg_pb2_grpc
from .hdz_grpc_msg.hdz_grpc_msg_pb2 import PoseStamped, Pose, Point, Quaternion, SetGripperRequest


class HdzGrpcArmClient:
    def __init__(self, host: str = "localhost", port: int = 9999):
        self.channel = grpc.insecure_channel(f"{host}:{port}")
        self.greeter_stub = hdz_grpc_msg_pb2_grpc.GreeterStub(self.channel)
        self.arm_stub = hdz_grpc_msg_pb2_grpc.ArmStub(self.channel)

    def __del__(self):
        self.channel.close()

    def MoveTo(self, pose_stamped: PoseStamped):
        response: PoseStamped = self.arm_stub.MoveTo(pose_stamped)
        return response

    def MoveToNamed(self, pose_name: str):
        response: PoseStamped = self.arm_stub.MoveToNamed(hdz_grpc_msg_pb2.StrMsg(str=pose_name))
        return response

    def SetGripper(
        self,
        normalized_width: float,
        max_effort: float = 10.0,
        grasp_depth: float = None, # Not used in the current implementation
    ):
        response: SetGripperRequest = self.arm_stub.SetGripper(
            hdz_grpc_msg_pb2.SetGripperRequest(
                normalized_width=normalized_width,
                max_effort=max_effort,
                grasp_depth=grasp_depth,
            )
        )
        return response
