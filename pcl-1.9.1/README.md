
# PCL-1.9.1 Modified Src Codes

First, download source code of [PCL-1.9.1](https://github.com/PointCloudLibrary/pcl/releases) Library. And then put the modified src code under the same directory of PCL-1.9.1 as shown in this repo. Finally, build PCL-1.9.1 library as follows:

```bash
cd <path-to-pcl-src-code>
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
```
