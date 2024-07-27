from concurrent import futures
import logging
import grpc
from hdz_grpc_msg import grasp_model_pb2, grasp_model_pb2_grpc, hdz_grpc_common_pb2


class Greeter(grasp_model_pb2_grpc.GreeterServicer):
    def SayHello(self, request, context):
        return hdz_grpc_common_pb2.StrMsg(str="Hello, %s!" % request.str)


def serve():
    port = "50051"
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    grasp_model_pb2_grpc.add_GreeterServicer_to_server(Greeter(), server)
    server.add_insecure_port("[::]:" + port)
    server.start()
    print("Server started, listening on " + port)
    # server.wait_for_termination()


def run():
    # NOTE(gRPC Python Team): .close() is possible on a channel and should be
    # used in circumstances in which the with statement does not fit the needs
    # of the code.
    print("Will try to greet world ...")
    with grpc.insecure_channel("localhost:50051") as channel:
        stub = grasp_model_pb2_grpc.GreeterStub(channel)
        response = stub.SayHello(hdz_grpc_common_pb2.StrMsg(str="you"))
    print("Greeter client received: " + response.str)


if __name__ == "__main__":
    logging.basicConfig()
    serve()
    run()
