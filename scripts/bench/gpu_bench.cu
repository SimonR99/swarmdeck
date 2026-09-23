// GPU benchmark: device-to-device bandwidth and cuBLAS SGEMM/HGEMM throughput.
// Build: nvcc -O2 gpu_bench.cu -lcublas -o gpu_bench
#include <cstdio>
#include <cublas_v2.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

#define CK(x) do { cudaError_t e = (x); if (e != cudaSuccess) { std::printf("cuda %s\n", cudaGetErrorString(e)); return 1; } } while (0)

int main() {
  cudaDeviceProp p;
  CK(cudaGetDeviceProperties(&p, 0));
  std::printf("gpu %s sm=%d.%d sms=%d\n", p.name, p.major, p.minor, p.multiProcessorCount);
  cudaEvent_t a, b;
  cudaEventCreate(&a);
  cudaEventCreate(&b);
  float ms;
  {
    size_t bytes = 512ull << 20;
    void *s, *d;
    CK(cudaMalloc(&s, bytes));
    CK(cudaMalloc(&d, bytes));
    cudaMemcpy(d, s, bytes, cudaMemcpyDeviceToDevice);
    cudaEventRecord(a);
    for (int i = 0; i < 10; ++i) cudaMemcpy(d, s, bytes, cudaMemcpyDeviceToDevice);
    cudaEventRecord(b);
    cudaEventSynchronize(b);
    cudaEventElapsedTime(&ms, a, b);
    std::printf("d2d_GBps %.1f\n", 10.0 * 2 * bytes / (ms / 1e3) / 1e9);
    cudaFree(s);
    cudaFree(d);
  }
  cublasHandle_t h;
  cublasCreate(&h);
  const int n = 4096;
  {
    float *A, *B, *C;
    CK(cudaMalloc(&A, sizeof(float) * n * n));
    CK(cudaMalloc(&B, sizeof(float) * n * n));
    CK(cudaMalloc(&C, sizeof(float) * n * n));
    cudaMemset(A, 0, sizeof(float) * n * n);
    cudaMemset(B, 0, sizeof(float) * n * n);
    float al = 1, be = 0;
    cublasSgemm(h, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, &al, A, n, B, n, &be, C, n);
    cudaEventRecord(a);
    for (int i = 0; i < 10; ++i)
      cublasSgemm(h, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, &al, A, n, B, n, &be, C, n);
    cudaEventRecord(b);
    cudaEventSynchronize(b);
    cudaEventElapsedTime(&ms, a, b);
    std::printf("sgemm4096_TFLOPS %.2f\n", 10.0 * 2.0 * n * n * n / (ms / 1e3) / 1e12);
    cudaFree(A); cudaFree(B); cudaFree(C);
  }
  {
    __half *A, *B, *C;
    CK(cudaMalloc(&A, 2ull * n * n));
    CK(cudaMalloc(&B, 2ull * n * n));
    CK(cudaMalloc(&C, 2ull * n * n));
    cudaMemset(A, 0, 2ull * n * n);
    cudaMemset(B, 0, 2ull * n * n);
    __half al = __float2half(1.f), be = __float2half(0.f);
    cublasHgemm(h, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, &al, A, n, B, n, &be, C, n);
    cudaEventRecord(a);
    for (int i = 0; i < 10; ++i)
      cublasHgemm(h, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, &al, A, n, B, n, &be, C, n);
    cudaEventRecord(b);
    cudaEventSynchronize(b);
    cudaEventElapsedTime(&ms, a, b);
    std::printf("hgemm4096_TFLOPS %.2f\n", 10.0 * 2.0 * n * n * n / (ms / 1e3) / 1e12);
  }
  return 0;
}
