# gRPC Server
This is a high-performance server that computes general and specialized point cloud algorithms.

Any client who has access to the proto would be able to construct a message and perform request to this server


# Build

- mkdir build && cd build
- cmake ..
- make -j4
- ./grpc/pc_server # <- executable.


# Docker Build
- `./docker_build.sh`

# Docker run
- `docker run -p 10001:10001 -it marcusforte/duna-cloud-processing`

# Compose with envoy
The docker compose will spin up the server as well as an Envoy proxy to translate HTTP requests to gRPC server. Default port: 5000
- `docker compose up`