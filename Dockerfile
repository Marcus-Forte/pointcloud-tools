FROM ubuntu:latest
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash

# Get dependencies.
RUN apt-get update && apt-get install git libpcl-dev build-essential cmake -y
RUN git clone --recurse-submodules -b v1.50.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc /deps/grpc
RUN mkdir -p /deps/grpc/build && cd /deps/grpc/build && \
    cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      .. && \
      make install -j6

COPY . /app
WORKDIR /app
# Build application.
RUN mkdir build && cd build && \
    cmake .. -DBUILD_GRPC=ON && \
    make -j4

# Run server.
CMD "/app/build/grpc/grpc-interface-server"
EXPOSE 10001