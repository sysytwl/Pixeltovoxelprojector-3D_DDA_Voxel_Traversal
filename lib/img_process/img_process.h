#include <Arduino.h>

class ImageProcessor {
public:
    void init(uint16_t width, uint16_t height, uint16_t block_width=3){
        _width = width;
        _height = height;
        _block_width = block_width;
        _result_width = (_width + _block_width - 1) / _block_width;
        _result_height = (_height + _block_width - 1) / _block_width;
        _result_size = ((_result_width * _result_height) + 7) / 8; // in bytes, for packed binary
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
        for (uint16_t y=0; y<_height; y+=block_width){
            for (uint16_t x=0; x<_width; x+=block_width){
                uint8_t tmpResult = 1;
                for (uint16_t j=0; j<block_width; j++) {//Compute OR for the 3x3 block centered at (x, y)
                    for (uint16_t i=0; i<=block_width; i++){
                        tmpResult = img[(y+j)*_width + (x+i)] & tmpResult;
                    }
                }
                for (uint16_t j=0; j<block_width; j++){
                    for (uint16_t i=0; i<=block_width; i++) {
                        img[(y+j)*_width + (x+i)] = tmpResult;
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
        // Clear output
        memset(img_out, 0, _result_size);
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
                uint32_t byte_idx = out_idx / 8;
                uint8_t bit_idx = out_idx % 8;
                if (edge) {
                    img_out[byte_idx] |= (1 << bit_idx);
                }
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
