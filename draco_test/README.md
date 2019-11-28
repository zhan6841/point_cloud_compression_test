test code for Draco library using kd-tree

```bash
# demo
cmake .
make
# lossy compression
./draco_test_compression ../data/0000000000.bin 10 10
python compare.py

# for lossless compression
./draco_test_compression ../data/0000000000.bin 10 18
```