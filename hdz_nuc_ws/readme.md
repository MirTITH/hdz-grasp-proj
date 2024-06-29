# HDZ NUC workspace

## Dependencies
```shell
# for grpc
sudo apt install -y build-essential autoconf libtool pkg-config
python3 -m pip install --upgrade pip
pip3 install grpcio grpcio-tools
```

## Build

### Build non-ROS packages
```shell
./build_non_ros_packages.sh
```

### Build colcon workspace packages
```shell
colcon build --symlink-install --mixin release compile-commands
```