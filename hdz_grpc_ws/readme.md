# HDZ gRPC workspace

这个目录下包含 gRPC 远程调用库和相应的消息定义。

## 安装方法

```shell
sudo apt install -y build-essential autoconf libtool pkg-config ninja-build
./build_install.sh
```

这个脚本会编译、安装下列包：

- grpc: gRPC 的 C++ 库
- hdz_grpc_msg: 撼地者使用的 gRPC 消息，会同时生成 C++ 和 Python 的库
- hdz_grpc_example: 示例，用于测试 hdz_grpc_msg 是否可用

其中：

- 编译中间文件会放在 ./build 文件夹下
- C++ 相关的库会安装在 ./install 文件夹下
- Python 库会使用 `python3 -m pip install <path>` 安装

## 卸载方法

```shell
./clean.sh
python3 -m pip uninstall hdz_grpc_msg
```

