import logging
import grpc
from hdz_grpc_msg import hdz_grpc_msg_pb2, hdz_grpc_msg_pb2_grpc
from hdz_grpc_msg.hdz_grpc_msg_pb2 import PoseStamped, Pose, Point, Quaternion


def run():
    # NOTE(gRPC Python Team): .close() is possible on a channel and should be
    # used in circumstances in which the with statement does not fit the needs
    # of the code.
    print("Will try to greet world ...")
    with grpc.insecure_channel("localhost:9999") as channel:
        stub = hdz_grpc_msg_pb2_grpc.GreeterStub(channel)
        response = stub.SayHello(hdz_grpc_msg_pb2.StrMsg(str="client example"))
        print("Greeter client received: " + response.str)

        move_grasp_stub = hdz_grpc_msg_pb2_grpc.MoveGraspStub(channel)
        move_grasp_response = move_grasp_stub.MoveTo(
            PoseStamped(
                frame_name="tool0",
                pose=Pose(
                    position=hdz_grpc_msg_pb2.Point(x=0, y=0, z=0.05),
                    orientation=hdz_grpc_msg_pb2.Quaternion(x=0, y=0, z=0, w=1),
                ),
            )
        )
        print(f"MoveGrasp client received: {move_grasp_response}")


if __name__ == "__main__":
    logging.basicConfig()
    run()
