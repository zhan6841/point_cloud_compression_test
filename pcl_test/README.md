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
* To configuration of the compression can be find in **pcl_test_compression.cpp** (line 229-250 or 148-157 or 69-78).