@echo off
REM Set the environment variables for vcpkg
set VCPKG_ROOT_DIR=D:\work\125.MyRobotics\vcpkg
set VCPKG_DEFAULT_TRIPLET=x64-windows-release

REM Create the build directory if it doesn't exist
if not exist build_windows mkdir build_windows

REM Change to the build directory
cd build_windows

REM Run CMake to configure the project
cmake -DG2O_BUILD_APPS=ON -DG2O_BUILD_EXAMPLES=ON -DG2O_USE_CHOLMOD=OFF -DG2O_USE_LGPL_LIBS=OFF -DVCPKG_TARGET_TRIPLET=%VCPKG_DEFAULT_TRIPLET% -DEIGEN3_INCLUDE_DIR="D:\work\125.MyRobotics\stuff\eigen" -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT_DIR%\scripts\buildsystems\vcpkg.cmake ..

REM Build the project
cmake --build . --target ALL_BUILD

REM Return to the original directory
cd ..