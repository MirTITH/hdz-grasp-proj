#!/bin/bash

script_dir=$(cd $(dirname $0); pwd)
grpc_dir=$script_dir/../hdz_grpc_msg/proto
grpc_proto_file=$grpc_dir/hdz_grpc_msg/helloworld.proto

# Generate gRPC code
python3 -m grpc_tools.protoc \
 -I $grpc_dir \
 --python_out=$script_dir \
 --pyi_out=$script_dir \
 --grpc_python_out=$script_dir \
 $grpc_proto_file

touch $script_dir/hdz_grpc_msg/__init__.py