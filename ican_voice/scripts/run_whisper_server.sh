#!/bin/bash
# Whisper Server Launcher with cuDNN fix
# Sets LD_LIBRARY_PATH for pip-installed nvidia-cudnn-cu12

# Find cuDNN library path
CUDNN_PATHS=(
    "$HOME/.local/lib/python3.10/site-packages/nvidia/cudnn/lib"
    "/usr/local/lib/python3.10/dist-packages/nvidia/cudnn/lib"
)

for path in "${CUDNN_PATHS[@]}"; do
    if [ -d "$path" ]; then
        echo "Found cuDNN at: $path"
        export LD_LIBRARY_PATH="$path:$LD_LIBRARY_PATH"
        break
    fi
done

# Also add other NVIDIA libraries
NVIDIA_LIB="$HOME/.local/lib/python3.10/site-packages/nvidia"
if [ -d "$NVIDIA_LIB" ]; then
    for lib_dir in "$NVIDIA_LIB"/*/lib; do
        if [ -d "$lib_dir" ]; then
            export LD_LIBRARY_PATH="$lib_dir:$LD_LIBRARY_PATH"
        fi
    done
fi

echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo ""
echo "Starting Whisper Server..."
exec ros2 run ican_voice whisper_server_node "$@"
