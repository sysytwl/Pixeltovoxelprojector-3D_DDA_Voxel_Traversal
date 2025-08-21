// rle_binary.hpp  –  drop straight into an ESP-IDF project
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace RLE {

/* -------- bit helpers (LSB-first packing) ------------------------------ */
static inline uint8_t getBit(const uint8_t* buf, size_t idx) {
    return (buf[idx >> 3] >> (idx & 7)) & 1U;
}

static inline void setBit(uint8_t* buf, size_t idx, uint8_t v) {
    const uint8_t mask = 1U << (idx & 7);
    if (v)  buf[idx >> 3] |=  mask;
    else    buf[idx >> 3] &= ~mask;
}

/* -------- encoder ------------------------------------------------------ */
/*
 * srcBits  : packed 1-bpp bitmap (LSB-first in every byte)
 * bitCount : number of valid bits in the bitmap
 * dst      : pre-allocated buffer, ≥ bitCount bytes (worst case)
 * returns  : number of bytes written to dst
 */
size_t encode(const uint8_t* srcBits, size_t bitCount, uint8_t* dst) {
    if (!bitCount) return 0;

    uint8_t curClr  = getBit(srcBits, 0);
    uint8_t runLen  = 0;
    size_t  outPos  = 0;

    for (size_t i = 0; i < bitCount; ++i) {
        uint8_t b = getBit(srcBits, i);
        if (b == curClr && runLen < 127) {
            ++runLen;
        } else {
            dst[outPos++] = (curClr << 7) | runLen;
            curClr = b;
            runLen = 1;
        }
    }
    /* flush last run */
    dst[outPos++] = (curClr << 7) | runLen;
    return outPos;
}

/* -------- decoder ------------------------------------------------------ */
/*
 * src      : RLE stream
 * srcSize  : its length (bytes)
 * dstBits  : destination bitmap (packed, LSB-first)
 * maxBits  : capacity of dstBits in *bits*
 * returns  : number of bits actually written
 */
size_t decode(const uint8_t* src, size_t srcSize,
              uint8_t* dstBits, size_t maxBits) {
    size_t bitIdx = 0;

    for (size_t i = 0; i < srcSize; ++i) {
        uint8_t byte  = src[i];
        uint8_t clr   = byte >> 7;
        uint8_t count = byte & 0x7F;

        for (uint8_t c = 0; c < count && bitIdx < maxBits; ++c)
            setBit(dstBits, bitIdx++, clr);
    }
    return bitIdx;         // caller can assert(bitIdx == expectedBits)
}

} // namespace RLE

#include "rle_binary.hpp"
#include <stdio.h>

void app_main() {
    constexpr size_t W = 128, H = 64;
    uint8_t bitmap[(W*H + 7)/8] = {0};

    /* make a white rectangle for the demo */
    for (size_t y = 10; y < 20; ++y)
        for (size_t x = 30; x < 90; ++x)
            RLE::setBit(bitmap, y*W + x, 1);

    /* worst-case output buffer = input bits (one RLE byte per pixel) */
    uint8_t rle[W*H];
    size_t  rleSize = RLE::encode(bitmap, W*H, rle);
    printf("compressed to %zu bytes (%.2f %% of raw)\n",
           rleSize, 100.0*rleSize/((W*H)/8));

    /* round-trip check */
    uint8_t roundTrip[(W*H + 7)/8] = {0};
    RLE::decode(rle, rleSize, roundTrip, W*H);

    /* compare ‑- trivial memcmp because both are packed */
    printf("round-trip %s\n",
           memcmp(bitmap, roundTrip, sizeof(bitmap)) == 0 ? "OK" : "FAIL");
}
