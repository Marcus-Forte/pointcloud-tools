FROM pytorch/pytorch:1.12.1-cuda11.3-cudnn8-devel

ENV TORCH_CUDA_ARCH_LIST="8.6"

RUN apt-get update && apt-get install -y unzip libgl1 git build-essential && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    pip3 install torch-points3d h5py


WORKDIR /root

RUN git clone https://github.com/HuguesTHOMAS/KPConv-PyTorch.git
