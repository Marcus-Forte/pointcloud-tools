FROM ubuntu:latest
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash

# Get dependencies.
RUN apt-get update && apt-get install git build-essential libeigen3-dev libflann-dev libboost-all-dev cmake -y

WORKDIR /deps

RUN git clone --recurse-submodules -b v1.56.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc && \
    git clone -b pcl-1.13.1 --depth 1 https://github.com/PointCloudLibrary/pcl.git && \
    git clone -b v1.2.1 --depth 1 https://github.com/Tessil/robin-map.git && \
    git clone -b v2021.10.0 --depth 1 https://github.com/oneapi-src/oneTBB.git && \
    git clone -b main https://github.com/Duna-System/duna-optimizer.git

RUN mkdir -p /deps/grpc/build && cd /deps/grpc/build && \
    cmake .. -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release && \
    make install -j$(nproc)

RUN mkdir -p /deps/pcl/build && cd /deps/pcl/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=OFF -DWITH_VTK=OFF -DBUILD_ml=OFF -DWITH_OPENGL=OFF && \
    make -j2 install

RUN mkdir -p /deps/oneTBB/build && cd /deps/oneTBB/build && \
    cmake .. -DMAKE_BUILD_TYPE=Release -DTBB_TEST=OFF && \
    make -j$(nproc) install

RUN mkdir -p /deps/robin-map/build && cd /deps/robin-map/build && \
    cmake .. -DMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) install

RUN mkdir -p /deps/duna-optimizer/build && cd /deps/duna-optimizer/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) install

RUN rm -rf /deps
COPY . /app
WORKDIR /app
# Build application.
RUN mkdir build && cd build && \
    cmake .. -DBUILD_GRPC=ON -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Run server.
CMD "/app/build/grpc/grpc-interface-server"
EXPOSE 10001
