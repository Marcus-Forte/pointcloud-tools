# gRPC Server
This is a high-performance server that computes general and specialized point cloud algorithms.

Any client who has access to the proto would be able to construct a message and perform request to this server


# Dev Env

We use Dev. Container extension to help get started with this repository. Make sure to download the extension and open this folder in the dev container.

Make sure to call `git submodule update --init` before build.

# Build

- mkdir build && cd build
- cmake ..
- make -j4
- ./grpc/pc_server # <- executable.

# Docker Build

- `docker build -f Dockerfile -t pointcloud-tools .`

# Docker run

To run `tools` with colmap, use the following. You probably need NVIDIA toolkit installed on the host (see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
