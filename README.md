# RoLabsSlam

RoLabsSlam is a Visual SLAM (Simultaneous Localization and Mapping) application with a core written in C++ and visualization implemented using C# Windows Forms. It uses OpenTK for 3D rendering and relies on several key dependencies, including OpenCVSharp for video processing and keypoint display, g2o for graph and camera pose optimization, and Eigen for linear algebra. The project is currently focused on monocular camera setups.

## Project Structure

```plaintext
RoLabsSlam/
├── .vs/                            # Visual Studio settings folder
├── opencv_files/                   # Prebuilt OpenCV 4 libraries from https://github.com/shimat/opencv_files
├── OpenCvSharp/                    # Wrapper library for OpenCV in C#, reference: https://github.com/shimat/opencvsharp
├── RoLabsSlam.Share/               # Visual slam implementation C++ code
├── RoLabsSlam.Windows/             # Exported DLL from C++ project
├── RoLabsSlam.Windows.Test/        # Main Windows Forms application
├── RoLabsSlamSharp/                # C# bindings and utilities
├── ThirdParty/                     # Additional third-party dependencies (e.g., g2o, Eigen)
├── .gitignore                      # Git ignore file
├── README.md                       # Project documentation (this file)
└── RoLabsSlam.sln                  # Visual Studio solution file
```

# Dependencies
## Core Dependencies

OpenCVSharp: A C# wrapper for OpenCV, used for reading video files and displaying keypoints in the Windows Forms application.
g2o: A general graph optimization framework used for optimizing camera poses and other SLAM-related calculations.
Eigen: A C++ template library for linear algebra, required by g2o and used for various mathematical operations in the SLAM core.

## Additional Dependencies
OpenCV 4 (via opencv_files): Prebuilt OpenCV libraries necessary for the SLAM core.
OpenTK: A library for 3D rendering in C# Windows Forms, used for visualizing the SLAM output.

# Getting Started
## Prerequisites

Visual Studio 2022 or later
vcpkg (optional, for managing additional dependencies)

### Building the Project
1. Clone the Repository
```bash
git clone https://github.com/yourusername/RoLabsSlam.git
cd RoLabsSlam
```

2. Configure and Build C++ Core
Download the prbuilt opencv libraries from https://github.com/shimat/opencv_files/releases and place it in RoLabsSlam\opencv_files\opencv4100_win_x64 </br>
Build with visual studio

3. Build the C# Application

Open RoLabsSlam.sln in Visual Studio.</br>
Restore NuGet packages.</br>
Build the solution to compile the C# Windows Forms application.</br>

Note: You need to rebuild RoLabsSlam.Windows.Test to run the test so that the C++ RoLabsSlam.Windows.dll and RoLabsSlam.Windows.pdb files are copied to the runtime folder during the post-build event when the C++ project is rebuilt. Alternatively, you can manually copy RoLabsSlam.Windows.dll and RoLabsSlam.Windows.pdb to RoLabsSlam.Windows.Test\bin\Debug\net8.0-windows10.0.xxxxx.0\runtimes\win-x64\native.

### Running the Application
After successfully building the project, you can run the Windows Forms application directly from Visual Studio. The application will read video files, process keypoints using the SLAM core, and display the results with 3D visualization.</br>

### License
This project is licensed under the MIT License - see the LICENSE file for details. For dependencies, please refer to their respective licenses.

### Acknowledgements
OpenCVSharp for providing an easy way to integrate OpenCV with C#.</br>
g2o and Eigen for their powerful optimization and mathematical tools.</br>
OpenTK for simplifying 3D rendering in C#.</br>
[ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) for indirect visual slam knowledge.</br>
[twitchslam](https://github.com/geohot/twitchslam) for local map points culling .</br>
