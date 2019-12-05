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
# lossy compression
./draco_test_compression ../data/0000000000.bin 10 10
python compare.py

# for lossless compression
./draco_test_compression ../data/0000000000.bin 10 18
```