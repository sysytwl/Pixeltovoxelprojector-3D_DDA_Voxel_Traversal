# Pixeltovoxelprojector
Projects motion of pixels to a voxel

# Reference:

1. Amanatides, J. and Woo, A., 1987, August. A fast voxel traversal algorithm for ray tracing. In Eurographics (Vol. 87, No. 3, pp. 3-10).

# Performances:

1. On ESP32 4MB PSRAM 800x600
``` cpp
PSRAM found. Size: 4192123 bytes
time cost for init cam ray lut: 979325
time cost for gray trans: 82285
time cost for binary: 82173
time cost for dissolve: 159224
time cost for edge dect: 163835
PSRAM used: 3801600 bytes, free: 390491 bytes
RAM used: 23600 bytes, free: 351976 bytes

speedup1:
PSRAM found. Size: 4192123 bytes
time cost for init cam ray lut: 979045
time cost for gray trans: 80377
time cost for binary: 76175
time cost for dissolve: 159285
time cost for edge dect: 117709
PSRAM used: 3801600 bytes, free: 390491 bytes
RAM used: 23600 bytes, free: 351976 bytes

speedup2:
PSRAM found. Size: 4192123 bytes
time cost for init cam ray lut: 979085
time cost for gray trans: 80379
time cost for binary: 56075
time cost for dissolve: 159273
time cost for edge dect: 117703
PSRAM used: 3801600 bytes, free: 390491 bytes
RAM used: 23600 bytes, free: 351976 bytes
```

2. on esp32s3 8mb spiflash 720p
```cpp
PSRAM found. Size: 8386295 bytes
time cost for init cam ray lut: 1556839
time cost for gray trans: 48642
time cost for binary: 45015
time cost for dissolve: 213547
time cost for edge dect: 145525
```