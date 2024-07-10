# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: grasp_model_grpc_msg/grasp_model_grpc_msg.proto
# Protobuf Python Version: 4.25.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n/grasp_model_grpc_msg/grasp_model_grpc_msg.proto\x12\x0chdz_grpc_msg\x1a\x1fgoogle/protobuf/timestamp.proto\"\xa1\x01\n\nPointCloud\x12-\n\ttimestamp\x18\x01 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\x12\n\nframe_name\x18\x02 \x01(\t\x12\x0e\n\x06height\x18\x03 \x01(\r\x12\r\n\x05width\x18\x04 \x01(\r\x12\x10\n\x08is_dense\x18\x05 \x01(\x08\x12\x0c\n\x04\x64\x61ta\x18\x06 \x01(\x0c\x12\x11\n\tuser_mask\x18\x07 \x01(\x0c\"\x15\n\x06StrMsg\x12\x0b\n\x03str\x18\x01 \x01(\t\"C\n\x0bPoseStamped\x12\x12\n\nframe_name\x18\x01 \x01(\t\x12 \n\x04pose\x18\x02 \x01(\x0b\x32\x12.hdz_grpc_msg.Pose\"\\\n\x04Pose\x12%\n\x08position\x18\x01 \x01(\x0b\x32\x13.hdz_grpc_msg.Point\x12-\n\x0borientation\x18\x02 \x01(\x0b\x32\x18.hdz_grpc_msg.Quaternion\"(\n\x05Point\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\"8\n\nQuaternion\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\x12\t\n\x01w\x18\x04 \x01(\x01\x32\x41\n\x07Greeter\x12\x36\n\x08SayHello\x12\x14.hdz_grpc_msg.StrMsg\x1a\x14.hdz_grpc_msg.StrMsg2[\n\nGraspModel\x12M\n\x16GenerateFromPointCloud\x12\x18.hdz_grpc_msg.PointCloud\x1a\x19.hdz_grpc_msg.PoseStampedb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'grasp_model_grpc_msg.grasp_model_grpc_msg_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_POINTCLOUD']._serialized_start=99
  _globals['_POINTCLOUD']._serialized_end=260
  _globals['_STRMSG']._serialized_start=262
  _globals['_STRMSG']._serialized_end=283
  _globals['_POSESTAMPED']._serialized_start=285
  _globals['_POSESTAMPED']._serialized_end=352
  _globals['_POSE']._serialized_start=354
  _globals['_POSE']._serialized_end=446
  _globals['_POINT']._serialized_start=448
  _globals['_POINT']._serialized_end=488
  _globals['_QUATERNION']._serialized_start=490
  _globals['_QUATERNION']._serialized_end=546
  _globals['_GREETER']._serialized_start=548
  _globals['_GREETER']._serialized_end=613
  _globals['_GRASPMODEL']._serialized_start=615
  _globals['_GRASPMODEL']._serialized_end=706
# @@protoc_insertion_point(module_scope)
