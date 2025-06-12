#include <Arduino.h>

class ImageProcessor {
public:
    void init(uint16_t width, uint16_t height, uint16_t block_width=3){
        _width = width;
        _height = height;
        _block_width = block_width;
        _result_width = (_width + _block_width - 1) / _block_width;
        _result_height = (_height + _block_width - 1) / _block_width;
        //_result_size = ((_result_width * _result_height) + 7) / 8; // in bytes, for packed binary
        _result_size = _result_width * _result_height;
    }

    void GrayToBinary(uint8_t* img, uint8_t threshold = 63) {
        uint8_t* end = img + _width * _height;
        while (img < end) {
            *img = (*img > threshold) ? 1 : 0;
            ++img;
        }
    }

    void dissolve(uint8_t* img) {
        uint16_t block_width = _block_width;
        for (uint16_t y = 0; y < _height; y += block_width) {
            for (uint16_t x = 0; x < _width; x += block_width) {
                uint8_t tmpResult = 1;
                uint16_t actual_block_h = ((y + block_width) > _height) ? (_height - y) : block_width;
                uint16_t actual_block_w = ((x + block_width) > _width) ? (_width - x) : block_width;
                // Compute AND for the block
                for (uint16_t j = 0; j < actual_block_h; j++) {
                    for (uint16_t i = 0; i < actual_block_w; i++) {
                        tmpResult &= img[(y + j) * _width + (x + i)];
                        if (tmpResult == 0) break; // Early exit if already zero
                    }
                    if (tmpResult == 0) break;
                }
                // Set all pixels in the block to tmpResult
                for (uint16_t j = 0; j < actual_block_h; j++) {
                    for (uint16_t i = 0; i < actual_block_w; i++) {
                        img[(y + j) * _width + (x + i)] = tmpResult;
                    }
                }
            }
        }
    }

    // img: input binary image (1 byte per pixel, 0/1)
    // img_out: output packed binary image (8 pixels per byte)
    void edge_dection(uint8_t* img, uint8_t* img_out){
        uint16_t bw = _block_width;
        uint16_t out_w = _result_width;
        uint16_t out_h = _result_height;
        for (uint16_t by = 0; by < out_h; ++by) {
            for (uint16_t bx = 0; bx < out_w; ++bx) {
                uint16_t y = by * bw;
                uint16_t x = bx * bw;
                uint16_t actual_block_h = ((y + bw) > _height) ? (_height - y) : bw;
                uint16_t actual_block_w = ((x + bw) > _width) ? (_width - x) : bw;
                uint32_t sum = 0;
                for (uint16_t j = 0; j < actual_block_h; ++j) {
                    for (uint16_t i = 0; i < actual_block_w; ++i) {
                        sum += img[(y + j) * _width + (x + i)];
                    }
                }
                uint32_t block_size = actual_block_h * actual_block_w;
                uint8_t edge = (sum == 0 || sum == block_size) ? 0 : 1;
                // Pack edge into output
                uint32_t out_idx = by * out_w + bx;
                img_out[out_idx] = edge;
            }
        }
    }

    uint16_t getResultWidth() const { return _result_width; }
    uint16_t getResultHeight() const { return _result_height; }
    uint32_t getResultSize() const { return _result_size; }

private:
    uint16_t _width, _height;
    uint16_t _block_width;
    uint16_t _result_width, _result_height;
    uint32_t _result_size;
};
