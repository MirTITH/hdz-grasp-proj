from google.protobuf import empty_pb2 as _empty_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SetGripperRequest(_message.Message):
    __slots__ = ("grasp_depth", "normalized_width", "max_effort")
    GRASP_DEPTH_FIELD_NUMBER: _ClassVar[int]
    NORMALIZED_WIDTH_FIELD_NUMBER: _ClassVar[int]
    MAX_EFFORT_FIELD_NUMBER: _ClassVar[int]
    grasp_depth: float
    normalized_width: float
    max_effort: float
    def __init__(self, grasp_depth: _Optional[float] = ..., normalized_width: _Optional[float] = ..., max_effort: _Optional[float] = ...) -> None: ...

class Array(_message.Message):
    __slots__ = ("data",)
    DATA_FIELD_NUMBER: _ClassVar[int]
    data: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, data: _Optional[_Iterable[float]] = ...) -> None: ...

class BoolWithMsg(_message.Message):
    __slots__ = ("is_success", "msg")
    IS_SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MSG_FIELD_NUMBER: _ClassVar[int]
    is_success: bool
    msg: str
    def __init__(self, is_success: bool = ..., msg: _Optional[str] = ...) -> None: ...

class StrMsg(_message.Message):
    __slots__ = ("str",)
    STR_FIELD_NUMBER: _ClassVar[int]
    str: str
    def __init__(self, str: _Optional[str] = ...) -> None: ...

class PoseStamped(_message.Message):
    __slots__ = ("frame_name", "pose")
    FRAME_NAME_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    frame_name: str
    pose: Pose
    def __init__(self, frame_name: _Optional[str] = ..., pose: _Optional[_Union[Pose, _Mapping]] = ...) -> None: ...

class Pose(_message.Message):
    __slots__ = ("position", "orientation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ORIENTATION_FIELD_NUMBER: _ClassVar[int]
    position: Point
    orientation: Quaternion
    def __init__(self, position: _Optional[_Union[Point, _Mapping]] = ..., orientation: _Optional[_Union[Quaternion, _Mapping]] = ...) -> None: ...

class Point(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...

class Quaternion(_message.Message):
    __slots__ = ("x", "y", "z", "w")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    W_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    w: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ..., w: _Optional[float] = ...) -> None: ...
