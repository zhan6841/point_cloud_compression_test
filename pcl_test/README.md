test code for PCL library using octree, you should build PCL-1.9.1 library following the instruction in **../pcl-1.9.1 directory** first before complie this code.

```bash
# demo
cmake .
make
./pcl_compression_test xyzi ../data/0000000000.bin
python compare.py
```

**NOTE**:

* The compression of the point position and color can be lossless or lossy while the compression of the **intensity** field is always lossless.
* The configuration of the compression can be find in **pcl_test_compression.cpp** (line 229-250 or 148-157 or 69-78). You can either use the default compression profile pre-defined in **compression_profiles.h** or use your manually defined compression profile.
* Currently, color coding and intensity coding **can not** be done at the same time.

## Explanation of PCL Compression Profiles

A compression profile consists of several fields as follows:

* **pointResolution**: This field is used for point position detail coding. The lower the value, the more accurate the point position after encoding&decoding. In other words, this field decides whether the compression of the point positions is lossy or lossless.

* **octreeResolution**: This field decides the minimum size of a voxel in an octree. 0.01 means 1mm.

* **doVoxelGridDownSampling**: This field allows coding for point attributes (e.g. position, color) details if it is set as false. Otherwise, the number of points in the original point cloud will be downsampled to the number of the voxels which will cause a decreasing of the number of points after compression.

* **iFrameRate**: There is an inter-frame compression algorithm implemented in PCL. By using it, consecutive point cloud frames can be encoded as I-frames and P-frames. This field allows users to set the iFrame Rate for inter-frame compression.

* **colorBitResolution**: This field is used for point color detail coding. The lower the value, the more accurate the point color after encoding&decoding. In other words, this field decides whether the compression of the point color is lossy or lossless.

* **doColorEncoding**: Enable color encoding if it is set as true. Otherwise, the point color attribute will be neglected.

* **doIntensityEncoding**: Enable intensity encoding if it is set as true. Otherwise, the point intensity attribute will be neglected.

For deeper understanding of PCL compression framework, please refer to this paper: "[Real-time Compression of Point Cloud Streams](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6224647)".
