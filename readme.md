# HDZ Grasp Project

How to clone this repo:

```shell
git clone --recurse-submodules --shallow-submodules https://github.com/MirTITH/hdz-grasp-proj.git
```

How to update submodules:

```shell
git submodule update --init --recursive --depth 1
```

## TODO

- [ ] 发布夹爪 normalized_width 话题
- [ ] 适配 MoveIt 到 Gazebo
- [ ] Gazebo 夹爪开闭速度限制
- [ ] 在不使用 Gazebo 时使用 `package://<package_name>` 在使用 Gazebo 时，使用 `file://$(find <package_name>)`
- [x] 修复手肘撞框架的问题
