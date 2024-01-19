# gRPC Server
This is a high-performance server that computes general and specialized point cloud algorithms.

Any client who has access to the proto would be able to construct a message and perform request to this server


# Build

- mkdir build && cd build
- cmake ..
- make -j4
- ./grpc/pc_server # <- executable.


# Docker Build

- `docker build -f Dockerfile -t duna-pointcloud-tools .`

If your PC does not have a CUDA device, swap the base image for `ubuntu:latest`:

- `docker build --build-arg BASE=ubuntu:latest -t duna-pointcloud-tools .`

Then, use the previously base image to build the colmap application.

- `docker build -f Dockerfile.colmap -t duna-pointcloud-tools-colmap .`

If building with GPU, make sure to set the cuda arch env accordingly. Example:

- `docker build -f Dockerfile.colmap --build-arg CUDA_ARCH=86 -t duna-pointcloud-tools-colmap .`

# Docker run

To run `tools` with colmap, use the following. You probably need NVIDIA toolkit installed on the host (see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

- `docker run -p 50052:50052 --gpus all -it duna-pointcloud-tools-colmap`

# Compose with envoy
The docker compose will spin up the server as well as an Envoy proxy to translate HTTP requests to gRPC server. Default port: 5000
- `docker compose up`