# We can swap for CUDA images to enable cuda usage
# nvidia base image: nvcr.io/nvidia/cuda:12.3.1-devel-ubi8

ARG BASE=nvcr.io/nvidia/cuda:12.2.2-devel-ubuntu22.04
ARG DEBIAN_FRONTEND=noninteractive

FROM ${BASE} as deps
ENV SHELL /bin/bash

# Get dependencies.
RUN apt-get update && apt-get install git build-essential clangd clang-tidy clang-format libeigen3-dev libflann-dev libboost-all-dev libgtest-dev cmake -y
 
WORKDIR /deps

RUN git clone --recurse-submodules -b v1.56.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc && \
    git clone -b pcl-1.13.1 --depth 1 https://github.com/PointCloudLibrary/pcl.git && \
    git clone -b v1.2.1 --depth 1 https://github.com/Tessil/robin-map.git && \
    git clone -b v2021.10.0 --depth 1 https://github.com/oneapi-src/oneTBB.git && \
    git clone -b main https://github.com/Marcus-Forte/moptimizer_0.git

RUN mkdir -p /deps/grpc/build && cd /deps/grpc/build && \
    cmake .. -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release && \
    make install -j$(nproc)

RUN mkdir -p /deps/pcl/build && cd /deps/pcl/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=OFF -DWITH_VTK=OFF -DBUILD_ml=OFF -DWITH_OPENGL=OFF && \
    make -j3 install

RUN mkdir -p /deps/oneTBB/build && cd /deps/oneTBB/build && \
    cmake .. -DMAKE_BUILD_TYPE=Release -DTBB_TEST=OFF && \
    make -j$(nproc) install

RUN mkdir -p /deps/robin-map/build && cd /deps/robin-map/build && \
    cmake .. -DMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) install

RUN mkdir -p /deps/moptimizer_0/build && cd /deps/moptimizer_0/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) install

# Build colmap
FROM deps as colmap

RUN apt-get update && apt-get install -y \
    libsqlite3-dev \
    libflann-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libcgal-dev \
    libglew-dev \
    qtbase5-dev \
    libceres-dev

RUN git clone https://github.com/colmap/colmap.git -b 3.9.1

# Arg may differ depending on the GPU
ARG CUDA_ARCH=86
RUN mkdir -p /deps/colmap/build && cd /deps/colmap/build && \
    cmake .. -DOPENGL_ENABLED=OFF -DGUI_ENABLED=OFF -DCUDA_ENABLED=ON -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} && \
    make -j$(nproc) install

FROM colmap as app
# Build application.
COPY . /app
WORKDIR /app

RUN mkdir build && cd build && \
    CI_BUILD=1 cmake .. -DBUILD_GRPC=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF && \
    make -j$(nproc)

# Cleanup
RUN rm -rf /deps

# Run server.
CMD "/app/build/grpc/grpc-interface-server"
EXPOSE 50052