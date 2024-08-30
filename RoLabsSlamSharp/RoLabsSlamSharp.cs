using System;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.InteropServices;
using OpenCvSharp;
using OpenCvSharp.Internal;
using OpenCvSharp.Internal.Vectors;

namespace RoLabsSlamSharp
{
    public class RolabsSlamSharpWrapper : IDisposable
    {
#if ANDROID
        private const string DllExtern = "libRoLabsSlam_Android.so";
#else
        private const string DllExtern = "RoLabsSlam.Windows";
#endif
        private IntPtr _slamPtr = IntPtr.Zero; // Pointer to native Slam object

        // P/Invoke for RoLabsFeatureExtraction_export
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern int RoLabsFeatureExtraction_export(
            IntPtr image, IntPtr keypoints);

        // P/Invoke for Slam_create
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern IntPtr Slam_create();

        // P/Invoke for Slam_destroy
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_destroy(IntPtr slam);

        // P/Invoke for Slam_grabImage
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_grabImage(IntPtr slam, IntPtr image);

        // P/Invoke for Slam_getDebugKeyPoints
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_getDebugKeyPoints(IntPtr slam, IntPtr keypointsCurrent, IntPtr keypointPrevious);

        // P/Invoke for Slam_start
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_start(IntPtr slam);

        // P/Invoke for Slam_stop
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_stop(IntPtr slam);

        // P/Invoke for Slam_setIntrinsicsMatrix
        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_setIntrinsicsMatrix(IntPtr slam, float fx, float fy, float cx, float cy);

        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_getCurrentPose(IntPtr slam, IntPtr pose);

        [DllImport(DllExtern, CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        private static extern void Slam_track(IntPtr slam);


        // Constructor - Create Slam object
        public RolabsSlamSharpWrapper()
        {
            _slamPtr = Slam_create();
            if (_slamPtr == IntPtr.Zero)
            {
                throw new Exception("Failed to create Slam instance.");
            }
        }

        // Destructor - Destroy Slam object
        ~RolabsSlamSharpWrapper()
        {
            Dispose(false);
        }

        // Dispose pattern to clean up unmanaged resources
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_slamPtr != IntPtr.Zero)
            {
                Slam_destroy(_slamPtr);
                _slamPtr = IntPtr.Zero;
            }
        }

        // GrabImage method
        public void GrabImage(Mat image)
        {
            if (_slamPtr == IntPtr.Zero)
                throw new ObjectDisposedException("Slam");

            Slam_grabImage(_slamPtr, image.CvPtr);
            GC.KeepAlive(image);
        }

        // GetDebugKeyPoints method
        public (KeyPoint[], KeyPoint[]) GetDebugKeyPoints()
        {
            if (_slamPtr == IntPtr.Zero)
                throw new ObjectDisposedException("Slam");

            using var keypointsVecCur = new VectorOfKeyPoint();
            using var keypointsVecPre = new VectorOfKeyPoint();
            Slam_getDebugKeyPoints(_slamPtr, keypointsVecCur.CvPtr, keypointsVecPre.CvPtr);

            KeyPoint[] keypointsCur = keypointsVecCur.ToArray();
            KeyPoint[] keypointsPre = keypointsVecPre.ToArray();

            return (keypointsCur, keypointsPre);
        }

        public Mat GetPose()
        {
            if (_slamPtr == IntPtr.Zero)
                throw new ObjectDisposedException("Slam");

            Mat pose = new Mat(4, 4, MatType.CV_64FC1);

            Slam_getCurrentPose(_slamPtr, pose.CvPtr);

            return pose;
        }

        // Stop the Slam process
        public void Stop()
        {
            if (_slamPtr == IntPtr.Zero)
                throw new ObjectDisposedException("Slam");

            Slam_stop(_slamPtr);
        }

        // Start the Slam process
        public void Start()
        {
            if (_slamPtr == IntPtr.Zero)
                throw new ObjectDisposedException("Slam");

            Slam_start(_slamPtr);
        }

        // Do tracking
        public void Track()
        {
            if (_slamPtr == IntPtr.Zero)
                throw new ObjectDisposedException("Slam");

            Slam_track(_slamPtr);
        }

        public void SetCameraIntrinsics(float fx, float fy, float cx, float cy)
        {
            if (_slamPtr == IntPtr.Zero)
                throw new ObjectDisposedException("Slam");
            Slam_setIntrinsicsMatrix(_slamPtr, fx, fy, cx, cy);
        }

        // RoLabsFeatureExtraction method
        public KeyPoint[] RoLabsFeatureExtraction(Mat image)
        {
            using var keypointsVec = new VectorOfKeyPoint();

            int keypointsCnts = RoLabsFeatureExtraction_export(image.CvPtr, keypointsVec.CvPtr);

            GC.KeepAlive(image);
            KeyPoint[] keypoints = keypointsVec.ToArray();
            return keypoints;
        }

        public void Destroy()
        {
            Dispose();
        }
    }
}
