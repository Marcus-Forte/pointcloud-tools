# Pointcloud processing interface

Similar to conversions, `duna-pointcloud-tools` offers tools for applying arbitrary processing algorithms to point clouds belonging to a volume shared with other applications.

Among the processing offered by this service, we can categorize as:

1. Metrics: They receive a cloud as input, and the output is a measurement (scalar). Examples:
  - Area
  - Volume

2. Unary: Receive a cloud as input, and the output can be a new cloud/object or a subset of the input cloud. Examples:
  - (subset) Voxel Grid
  - (subset) Removal of statistical outliers
  - (new object) 3D Mesh

3. Binaries: Receive two input clouds, and the output could be transformation matrices or new pointclouds. Examples:
  - (transform) Registration.
  - (new cloud) Concatenation

4. Image: Receives a set of input file path images processed into a 3D model.  