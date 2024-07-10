#!/bin/bash

script_dir=$(cd $(dirname $0); pwd)
# grpc_dir=$script_dir
grpc_proto_file=$script_dir/hdz_grpc_msg/hdz_grpc_msg.proto

# Generate gRPC code
python3 -m grpc_tools.protoc \
 -I $script_dir \
 --python_out=$script_dir \
 --pyi_out=$script_dir  \
 --grpc_python_out=$script_dir  \
 $grpc_proto_file
