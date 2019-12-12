
# PCL-1.9.1 Modified Src Codes

First, download source code of [PCL-1.9.1](https://github.com/PointCloudLibrary/pcl/releases) Library. And then put the modified src code under the same directory of PCL-1.9.1 as shown in this repo. Finally, build PCL-1.9.1 library as follows:

```bash
cd <path-to-pcl-src-code>
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
```

The build of PCL library will be very slow, keep patient.

If you want to understand the compression framework in PCL, you can refer to this paper "[Real-time Compression of Point Cloud Streams](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6224647)".

The compression algorithm that is implemented in **intensity_coding.h** is almost the same as that in **color_coding.h**. The difference is that first, the *Intensity Bit Resolution* is always set as 8 while the *Color Bit Resolution* can be set in [0, 8]. This is to make sure the compression of *intensity* will always be lossless. Second, when compressing *color*, **color_coding.h** will drop the 24-31 bits of every *color* because in PCL, *XYZRGB/XYZRGBA class* of point cloud use a float to represent the *color* of one point (0-7 bits for R, 8-15 bits for G and 16-23 bits for B). However, for *intensity* compression, all the bits of a float need to be kept.
