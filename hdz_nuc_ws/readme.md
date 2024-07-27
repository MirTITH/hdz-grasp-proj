# HDZ NUC workspace

## Dependencies
```shell
# for ros workspace
rosdep install --from-paths src --ignore-src -r -y
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