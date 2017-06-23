#include "visual_frontend/gpu.h"

namespace visual_frontend { namespace gpu {

#ifdef OPENCV_CUDA

  //
  // These methods are used to download data from device (GPU) to host (CPU)
  //

  // https://github.com/opencv/opencv/blob/master/samples/gpu/pyrlk_optical_flow.cpp
  void download(const cv::cuda::GpuMat& d_mat, std::vector<cv::Point2f>& vec)
  {
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
    d_mat.download(mat);
  }

  void download(const cv::cuda::GpuMat& d_mat, std::vector<uchar>& vec)
  {
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
    d_mat.download(mat);
  }

  //
  // These methods are used to update data from host (CPU) to device (GPU)
  //

  void upload(const std::vector<cv::Point2f>& vec, cv::cuda::GpuMat& gMat)
  {
    cv::Mat tmp(1, vec.size(), CV_32FC2, (void*)&vec[0]);
    gMat.upload(tmp);
  }

#endif

}}