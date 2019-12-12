test code for Draco library using kd-tree, you should build Draco library first before complie this code.

```bash
# build Draco
cd ../draco
mkdir build
cd build
cmake ../
make
# demo
cmake .
make
# Usage: ./draco_test_compression [input_file(.bin)] cl(compression level:0-10) qb(quantization bits:1-31)
# lossy compression
./draco_test_compression ../data/0000000000.bin 10 10
python compare.py

# for lossless compression
./draco_test_compression ../data/0000000000.bin 10 18
```

**NOTE**:

Draco currently supports following types of point cloud attributes:

* **POSITION**

* **NORMAL**

* **COLOR**

* **TEX_COORD**

* **GENERIC**: A special id used to mark attributes that are not assigned to any known predefined use case. Such attributes are often used for a shader specific data. **We use this type of attributes for intensity.**

## Explanation of Draco Compression Configurations

There are several fields that will impact on the compression performance of Draco as follows:

* **cl(compression level:0-10)**: This field decides the encoding/decoding speed (speed = 10 - cl). According to the comments in Draco src code, cl = 10 means the slowest speed, but the best compression. cl = 0 means the fastest, but the worst compression. Note that both speed options (encoding speed & decoding speed) affect the encoder choice of used methods and algorithms. For example, a requirement for fast decoding may prevent the encoder from using the best compression methods even if the encoding speed is set to 0. In general, the faster of the two options limits the choice of features that can be used by the encoder. Additionally, setting |decoding_speed| to be faster than the |encoding_speed| may allow the encoder to choose the optimal method out of the available features for the given |decoding_speed|. **For KD-tree based compression, the cl from 6 to 10 will have the same performance because Draco currently do not have a encoding algorithm which performs better than that when cl = 6.**

* **qb(quantization bits:1-31)**: This field decides the quantization compression options for all attributes of a point cloud. According to the comments in Draco src code, the attribute values will be quantized in a box defined by the maximum extent of the attribute values. I.e., the actual precision of this option depends on the scale of the attribute values. **In my implementation, the same quantization bits is used for all types of attributes. But you can also use different quantization bits for different types of attributes by modifying the code.**
