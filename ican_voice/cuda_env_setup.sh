# CUDA/cuDNN Library Path Configuration
# Add this to your ~/.bashrc to make it permanent

# Add pip-installed NVIDIA CUDA libraries to LD_LIBRARY_PATH
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.10/site-packages/nvidia/cudnn/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.10/site-packages/nvidia/cublas/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.10/site-packages/nvidia/cuda_runtime/lib:$LD_LIBRARY_PATH"

# Or use this more comprehensive approach:
# for lib in $HOME/.local/lib/python3.10/site-packages/nvidia/*/lib; do
#     [ -d "$lib" ] && export LD_LIBRARY_PATH="$lib:$LD_LIBRARY_PATH"
# done
