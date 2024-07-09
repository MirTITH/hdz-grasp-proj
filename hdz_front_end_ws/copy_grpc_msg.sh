#!/bin/bash

script_path=$(readlink -f "$0")        # 脚本文件的绝对路径
script_dir=$(dirname "$script_path")   # 脚本文件的目录

dst_path=$script_dir/src/grasp_model_grpc_msg/
src_path=$script_dir/../grasp_model/grasp_model_grpc_msg

cp -r $src_path $dst_path