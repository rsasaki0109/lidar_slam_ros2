#include <gpu_bbs3d/stream_manager/check_error.cuh>

namespace gpu {

void CUDACheckError::operator<<(cudaError_t error) const {
  if (error == cudaSuccess) {
    return;
  }

  const std::string error_name = cudaGetErrorName(error);
  const std::string error_string = cudaGetErrorString(error);

  std::cerr << "warning: " << error_name << std::endl;
  std::cerr << "       : " << error_string << std::endl;
}

CUDACheckError check_error;

}  // namespace gpu