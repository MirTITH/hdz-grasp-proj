#!/bin/bash
script_dir=$(cd $(dirname $0); pwd)

rm -rf $script_dir/non_ros_pkgs/build
rm -rf $script_dir/non_ros_pkgs/install

