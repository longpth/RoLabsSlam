using System;
using OpenTK.Mathematics;
using OpenCvSharp;

namespace RoLabsSlam.Windows.Test
{
    public static class MatExtensions
    {
        public static Matrix4 ToMatrix4(this Mat mat)
        {
            // Check if the Mat is 3x4 or 4x4
            if ((mat.Rows != 3 && mat.Rows != 4) || mat.Cols != 4)
            {
                throw new ArgumentException("The input Mat must be a 3x4 or 4x4 matrix.");
            }

            Matrix4 OpenGLMatrix;

            if (mat.Depth() == MatType.CV_32F) // float (32-bit)
            {
                OpenGLMatrix = new Matrix4(
                    mat.At<float>(0, 0), mat.At<float>(0, 1), mat.At<float>(0, 2), mat.At<float>(0, 3),
                    mat.At<float>(1, 0), mat.At<float>(1, 1), mat.At<float>(1, 2), mat.At<float>(1, 3),
                    mat.At<float>(2, 0), mat.At<float>(2, 1), mat.At<float>(2, 2), mat.At<float>(2, 3),
                    mat.Rows == 4 ? mat.At<float>(3, 0) : 0f, // Check if the Mat has 4 rows
                    mat.Rows == 4 ? mat.At<float>(3, 1) : 0f,
                    mat.Rows == 4 ? mat.At<float>(3, 2) : 0f,
                    mat.Rows == 4 ? mat.At<float>(3, 3) : 1f // The last row [0, 0, 0, 1]
                );
            }
            else if (mat.Depth() == MatType.CV_64F) // double (64-bit)
            {
                OpenGLMatrix = new Matrix4(
                    (float)mat.At<double>(0, 0), (float)mat.At<double>(0, 1), (float)mat.At<double>(0, 2), (float)mat.At<double>(0, 3),
                    (float)mat.At<double>(1, 0), (float)mat.At<double>(1, 1), (float)mat.At<double>(1, 2), (float)mat.At<double>(1, 3),
                    (float)mat.At<double>(2, 0), (float)mat.At<double>(2, 1), (float)mat.At<double>(2, 2), (float)mat.At<double>(2, 3),
                    mat.Rows == 4 ? (float)mat.At<double>(3, 0) : 0f, // Check if the Mat has 4 rows
                    mat.Rows == 4 ? (float)mat.At<double>(3, 1) : 0f,
                    mat.Rows == 4 ? (float)mat.At<double>(3, 2) : 0f,
                    mat.Rows == 4 ? (float)mat.At<double>(3, 3) : 1f // The last row [0, 0, 0, 1]
                );
            }
            else
            {
                throw new NotSupportedException("The input Mat must be of type CV_32F (float) or CV_64F (double).");
            }

            return OpenGLMatrix;
        }

    }
}
