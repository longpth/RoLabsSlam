export NDK_ROOT=/home/longpth/workspace/work/001.AndroidStudio/sdk/ndk/27.0.12077973
cmake -DCMAKE_TOOLCHAIN_FILE=${NDK_ROOT}/build/cmake/android.toolchain.cmake \
      -DANDROID_NDK=${NDK_ROOT}                               \
      -DANDROID_ABI="arm64-v8a"                    \
      -DEIGEN3_INCLUDE_DIR="/home/longpth/workspace/work/125.MyRobotics/stuff/eigen" \
      -DEIGEN3_VERSION_OK=ON ..