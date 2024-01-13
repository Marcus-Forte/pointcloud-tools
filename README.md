# gRPC Server
This is a high-performance server that computes general and specialized point cloud algorithms.

Any client who has access to the proto would be able to construct a message and perform request to this server


# Build

- mkdir build && cd build
- cmake ..
- make -j4
- ./grpc/pc_server # <- executable.


# Docker Build
NOTE: If your PC does not have a CUDA device, swap the base image for `ubuntu:latest`

- `./docker_build.sh` or `docker build duna-pointcloud-tools .`

To build `tools` with colmap, use the following. Make sure base image is built.

- `docker build -f Dockerfile -t duna-pointcloud-tools-gpu .`
- `docker build -f Dockerfile.colmap -t duna-pointcloud-tools-colmap .`

# Docker run
- `docker run -p 50052:50052 -it duna-pointcloud-tools`

To run `tools` with colmap, use the following. You probably need NVIDIA toolkit installed on the host (see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

- `docker run -p 50052:50052 --gpus all -it duna-pointcloud-tools-colmap`

# Compose with envoy
The docker compose will spin up the server as well as an Envoy proxy to translate HTTP requests to gRPC server. Default port: 5000
- `docker compose up`