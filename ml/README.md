# Environment

- Navitage to `/app/KPConv-PyTorch/cpp_wrappers` in the devcontainer
- run `compile_wrappers.sh`
- `cd ~`
- Extract .zip models and dataset inside docker container:
  - `unzip /workspaces/pointcloud-tools/ml/Stanford3dDataset_v1.2.zip` <-- Dataset 
  - `unzip /workspaces/pointcloud-tools/ml/Light_KPFCNN.zip` <-- Model

- code `/home/KPConv-PyTorch/`
  - in `test_models.py` set  `chosen_log = /root/Light_KPFCNN`
  - in `S3DIS.py` set `self.path = `

# Resources

- [Pretrained Model](https://drive.google.com/file/d/14sz0hdObzsf_exxInXdOIbnUTe0foOOz/view?usp=sharing)
- [Classification Dataset](https://goo.gl/forms/4SoGp4KtH1jfRqEj2)
- [Classification Dataset](https://shapenet.cs.stanford.edu/media/modelnet40_normal_resampled.zip)

- https://arxiv.org/abs/1904.08889
- https://github.com/HuguesTHOMAS/KPConv-PyTorch