# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc
import warnings

from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2
from hdz_grpc_msg import hdz_grpc_msg_pb2 as hdz__grpc__msg_dot_hdz__grpc__msg__pb2

GRPC_GENERATED_VERSION = '1.64.1'
GRPC_VERSION = grpc.__version__
EXPECTED_ERROR_RELEASE = '1.65.0'
SCHEDULED_RELEASE_DATE = 'June 25, 2024'
_version_not_supported = False

try:
    from grpc._utilities import first_version_is_lower
    _version_not_supported = first_version_is_lower(GRPC_VERSION, GRPC_GENERATED_VERSION)
except ImportError:
    _version_not_supported = True

if _version_not_supported:
    warnings.warn(
        f'The grpc package installed is at version {GRPC_VERSION},'
        + f' but the generated code in hdz_grpc_msg/hdz_grpc_msg_pb2_grpc.py depends on'
        + f' grpcio>={GRPC_GENERATED_VERSION}.'
        + f' Please upgrade your grpc module to grpcio>={GRPC_GENERATED_VERSION}'
        + f' or downgrade your generated code using grpcio-tools<={GRPC_VERSION}.'
        + f' This warning will become an error in {EXPECTED_ERROR_RELEASE},'
        + f' scheduled for release on {SCHEDULED_RELEASE_DATE}.',
        RuntimeWarning
    )


class GreeterStub(object):
    """Service definition.

    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.SayHello = channel.unary_unary(
                '/hdz_grpc_msg.Greeter/SayHello',
                request_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.SerializeToString,
                response_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.FromString,
                _registered_method=True)


class GreeterServicer(object):
    """Service definition.

    """

    def SayHello(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_GreeterServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'SayHello': grpc.unary_unary_rpc_method_handler(
                    servicer.SayHello,
                    request_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.FromString,
                    response_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'hdz_grpc_msg.Greeter', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
    server.add_registered_method_handlers('hdz_grpc_msg.Greeter', rpc_method_handlers)


 # This class is part of an EXPERIMENTAL API.
class Greeter(object):
    """Service definition.

    """

    @staticmethod
    def SayHello(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/hdz_grpc_msg.Greeter/SayHello',
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.SerializeToString,
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)


class ArmStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.MoveTo = channel.unary_unary(
                '/hdz_grpc_msg.Arm/MoveTo',
                request_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.PoseStamped.SerializeToString,
                response_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
                _registered_method=True)
        self.MoveToJointPosition = channel.unary_unary(
                '/hdz_grpc_msg.Arm/MoveToJointPosition',
                request_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.Array.SerializeToString,
                response_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
                _registered_method=True)
        self.MoveToNamed = channel.unary_unary(
                '/hdz_grpc_msg.Arm/MoveToNamed',
                request_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.SerializeToString,
                response_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
                _registered_method=True)
        self.SetGripper = channel.unary_unary(
                '/hdz_grpc_msg.Arm/SetGripper',
                request_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.SetGripperRequest.SerializeToString,
                response_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
                _registered_method=True)
        self.GetPose = channel.unary_unary(
                '/hdz_grpc_msg.Arm/GetPose',
                request_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.SerializeToString,
                response_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.PoseStamped.FromString,
                _registered_method=True)
        self.GetJointPosition = channel.unary_unary(
                '/hdz_grpc_msg.Arm/GetJointPosition',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.Array.FromString,
                _registered_method=True)


class ArmServicer(object):
    """Missing associated documentation comment in .proto file."""

    def MoveTo(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def MoveToJointPosition(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def MoveToNamed(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SetGripper(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetPose(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetJointPosition(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_ArmServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'MoveTo': grpc.unary_unary_rpc_method_handler(
                    servicer.MoveTo,
                    request_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.PoseStamped.FromString,
                    response_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.SerializeToString,
            ),
            'MoveToJointPosition': grpc.unary_unary_rpc_method_handler(
                    servicer.MoveToJointPosition,
                    request_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.Array.FromString,
                    response_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.SerializeToString,
            ),
            'MoveToNamed': grpc.unary_unary_rpc_method_handler(
                    servicer.MoveToNamed,
                    request_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.FromString,
                    response_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.SerializeToString,
            ),
            'SetGripper': grpc.unary_unary_rpc_method_handler(
                    servicer.SetGripper,
                    request_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.SetGripperRequest.FromString,
                    response_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.SerializeToString,
            ),
            'GetPose': grpc.unary_unary_rpc_method_handler(
                    servicer.GetPose,
                    request_deserializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.FromString,
                    response_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.PoseStamped.SerializeToString,
            ),
            'GetJointPosition': grpc.unary_unary_rpc_method_handler(
                    servicer.GetJointPosition,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=hdz__grpc__msg_dot_hdz__grpc__msg__pb2.Array.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'hdz_grpc_msg.Arm', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
    server.add_registered_method_handlers('hdz_grpc_msg.Arm', rpc_method_handlers)


 # This class is part of an EXPERIMENTAL API.
class Arm(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def MoveTo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/hdz_grpc_msg.Arm/MoveTo',
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.PoseStamped.SerializeToString,
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def MoveToJointPosition(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/hdz_grpc_msg.Arm/MoveToJointPosition',
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.Array.SerializeToString,
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def MoveToNamed(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/hdz_grpc_msg.Arm/MoveToNamed',
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.SerializeToString,
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def SetGripper(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/hdz_grpc_msg.Arm/SetGripper',
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.SetGripperRequest.SerializeToString,
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.BoolWithMsg.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def GetPose(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/hdz_grpc_msg.Arm/GetPose',
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.StrMsg.SerializeToString,
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.PoseStamped.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def GetJointPosition(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/hdz_grpc_msg.Arm/GetJointPosition',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            hdz__grpc__msg_dot_hdz__grpc__msg__pb2.Array.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)