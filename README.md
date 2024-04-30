# gRPC Server

This is a high-performance server that computes general and specialized point cloud algorithms.
Any client who has access to the proto would be able to construct a message and perform request to this server

## Dev Env

We use Dev. Container extension to help get started with this repository. Make sure to download the Dev. Container extension and open this folder in the dev container.

## Build

Simply use container cmake extension to build the application. Or call `cmake .. && make` from `/app/build` inside the container.
Make sure to call `git submodule update --init` before build.

## Test

Simply use container cmake extension to test the application. Or call `ctest` from `/app/build` inside the container.

## Docker build

- `docker build . -t pointcloud-tools`

## Docker run

- `docker run -p 50052:50052 pointcloud-tools`
- Call any gRPC method.

## Notes

You need NVIDIA toolkit installed on the host (see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to be able to use colmap with GPU.
