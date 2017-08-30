#pragma once

#ifdef OPENCV_CUDA
  #include <opencv2/cudaoptflow.hpp>
  #include <opencv2/cudaimgproc.hpp>
  #include <opencv2/cudaarithm.hpp>
  #include <opencv2/cudafilters.hpp>
  #include <opencv2/cudawarping.hpp>
#endif

namespace visual_frontend { namespace gpu {

#ifdef OPENCV_CUDA

  //
  // These methods are used to download data from device (GPU) to host (CPU)
  //

  // https://github.com/opencv/opencv/blob/master/samples/gpu/pyrlk_optical_flow.cpp
  void download(const cv::cuda::GpuMat& d_mat, std::vector<cv::Point2f>& vec);
  void download(const cv::cuda::GpuMat& d_mat, std::vector<uchar>& vec);

  //
  // These methods are used to update data from host (CPU) to device (GPU)
  //

  void upload(const std::vector<cv::Point2f>& vec, cv::cuda::GpuMat& gMat);

#endif

}}