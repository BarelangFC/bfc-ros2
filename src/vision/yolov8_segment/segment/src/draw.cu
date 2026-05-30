#include "draw.h"
#include "utils.h"
#include "config.h"

__global__ void draw_mask_kernel(uchar* imgData, float* mask, int h, int w, int color_b, int color_g, int color_r) {
    int ix = threadIdx.x + blockIdx.x * blockDim.x;
    int iy = threadIdx.y + blockIdx.y * blockDim.y;
    int idx = ix + iy * w;
    int idx3 = idx * 3;

    if (ix >= w || iy >= h) return;

    if (mask[idx] > 0.5) {
        imgData[idx3] = static_cast<uchar>(imgData[idx3] * 0.5 + color_b * 0.5);
        imgData[idx3 + 1] = static_cast<uchar>(imgData[idx3 + 1] * 0.5 + color_g * 0.5);
        imgData[idx3 + 2] = static_cast<uchar>(imgData[idx3 + 2] * 0.5 + color_r * 0.5);
    }
}

void draw_mask(cv::Mat& img, float* mask, int classId) {
    int h = img.rows;
    int w = img.cols;
    int wh = h * w;
    int elements = wh * 3;

    uchar* imgDataDevice;
    cudaMalloc((void**)&imgDataDevice, elements * sizeof(uchar));
    cudaMemcpy(imgDataDevice, img.data, elements * sizeof(uchar), cudaMemcpyHostToDevice);

    float* maskDevice;
    cudaMalloc((void**)&maskDevice, wh * sizeof(float));
    cudaMemcpy(maskDevice, mask, wh * sizeof(float), cudaMemcpyHostToDevice);

    int color_b, color_g, color_r;

    // Assign static colors based on class ID
    if (classId == 0) {  // Ball
        color_b = kBallColor[0];
        color_g = kBallColor[1];
        color_r = kBallColor[2];
    } /*else if (classId == 1) {  // field
        color_b = kFieldColor[0];
        color_g = kFieldColor[1];
        color_r = kFieldColor[2];
    } */else {
        // Default fallback color
        color_b = 128;
        color_g = 128;
        color_r = 128;
    }
    
    dim3 blockSize(32, 32);
    dim3 gridSize((w + blockSize.x - 1) / blockSize.x, (h + blockSize.y - 1) / blockSize.y);

    draw_mask_kernel<<<gridSize, blockSize>>>(imgDataDevice, maskDevice, h, w, color_b, color_g, color_r);

    cudaMemcpy(img.data, imgDataDevice, elements * sizeof(uchar), cudaMemcpyDeviceToHost);

    cudaFree(imgDataDevice);
    cudaFree(maskDevice);
}
