#include <Arduino.h>

class ImageProcessor {
public:
    void init(uint16_t width, uint16_t height){
        _width = width;
        _height = height;
    }

    void YUV422ToGray(uint8_t* img) {
        for (size_t i = 0; i < _width * _height; i++) {
            img[i] = img[i] >> 4;
        }
    }

    void GrayToBinary(uint8_t* img, uint8_t threshold = 63) {
        for (size_t i = 0; i < _width*_height; i++) {
            img[i] = (img[i] > threshold) ? 1 : 0;
        }
    }

    void dissolve(uint8_t* img, uint16_t block_width=3) {
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

    void edge_dection(uint8_t* img, uint16_t block_width=4){
        for (uint16_t y=0; y<_height; y+=block_width){
            size_t tmp_blockline=y*_width;
            for (uint16_t x=0; x<_width; x+=block_width){
                size_t tmp_block = tmp_blockline + x;
                uint32_t xorResult = 0;
                for (uint16_t j=0; j<block_width; j++) {
                    size_t tmpline = tmp_block + j*_width + x;
                    for (uint16_t i=0; i<=block_width; i++){
                        xorResult += img[tmpline+i];
                    }
                }
                xorResult = (xorResult >= 16)||(xorResult==0) ? 0 : 1;
                for (uint16_t j=0; j<block_width; j++){
                    size_t tmpline = tmp_block + j*_width + x;
                    for (uint16_t i=0; i<=block_width; i++) {
                        img[tmpline+i] = xorResult;
                    }
                }
            }
        }
    }

private:
    uint16_t _width, _height;
};
