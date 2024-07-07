import logging
import grpc
from hdz_grpc_msg import hdz_grpc_msg_pb2, hdz_grpc_msg_pb2_grpc
from hdz_grpc_msg.hdz_grpc_msg_pb2 import PoseStamped, Pose, Point, Quaternion, SetGripperRequest


def run():
    # NOTE(gRPC Python Team): .close() is possible on a channel and should be
    # used in circumstances in which the with statement does not fit the needs
    # of the code.
    print("Will try to greet world ...")
    with grpc.insecure_channel("192.168.1.150:9999") as channel:
        stub = hdz_grpc_msg_pb2_grpc.GreeterStub(channel)
        response = stub.SayHello(hdz_grpc_msg_pb2.StrMsg(str="client example"))
        print(f"Greeter client received: " + response.str)

        arm_stub = hdz_grpc_msg_pb2_grpc.ArmStub(channel)
        # arm_response = arm_stub.MoveTo(
        #     PoseStamped(
        #         frame_name="tool0",
        #         pose=Pose(
        #             position=hdz_grpc_msg_pb2.Point(x=0, y=0, z=0.05),
        #             orientation=hdz_grpc_msg_pb2.Quaternion(x=0, y=0, z=0, w=1),
        #         ),
        #     )
        # )
        # print(f"Arm client received: {arm_response}")

        gripper_response = arm_stub.SetGripper(
            SetGripperRequest(
                normalized_width=0.5,
                max_effort=0.01,
            )
        )
        print(f"Gripper client received: {gripper_response}")


if __name__ == "__main__":
    logging.basicConfig()
    run()
