#include <torch/torch.h>
#include <iostream>

int main() {
    // Check if CUDA is available
    if(torch::cuda::is_available()){
        std::cout << "CUDA is available! Using GPU." << std::endl;
        // Create a tensor directly on the GPU
        torch::Tensor tensor = torch::randn({2, 3}, torch::device(torch::kCUDA));
        std::cout << tensor << std::endl;
    } else {
        std::cout << "CUDA is not available. Using CPU." << std::endl;
        // Create a tensor on the CPU (default)
        torch::Tensor tensor = torch::randn({2, 3});
        std::cout << tensor << std::endl;
    }
    return 0;
}