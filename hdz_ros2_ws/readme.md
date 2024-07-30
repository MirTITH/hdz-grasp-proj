# HDZ NUC workspace

## Install dependencies
```shell
# colcon mixin (Optional)
sudo apt install python3-colcon-common-extensions python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# ROS 2 workspace dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Build

### Build colcon workspace packages
```shell
colcon build --symlink-install --mixin release compile-commands
```

If you didn't install mixin, use the following command instead:
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli
```

