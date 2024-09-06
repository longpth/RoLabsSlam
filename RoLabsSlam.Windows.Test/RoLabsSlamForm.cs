using OpenCvSharp;
using RoLabsSlamSharp;
using System;
using System.Windows.Forms;
//using SharpDX;
//using SharpDX.Direct3D11;
//using SharpDX.DXGI;
//using SharpDX.Direct3D;
//using Device = SharpDX.Direct3D11.Device;

using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.WinForms;
using RoLabsSlam.Windows.Test;
using System.Diagnostics;
using static System.Net.Mime.MediaTypeNames;

namespace RoLabsSlam.Test
{
    enum CameraState
    {
        IsStopped = 1, IsStarted = 2, IsPaused = 3,
    }
    public partial class RoLabsSlamForm : Form
    {
        private readonly string videoPath = @"\..\..\..\..\..\dataset\kitti_sequence_00.mp4";

        private readonly float VisualizedScaled = 20.0f;

        private VideoCapture _videoCapture;
        private Mat _frame;
        private System.Windows.Forms.Timer _timer;
        private RolabsSlamSharpWrapper _rolabsSlamWrapper;
        private CameraState _cameraState = CameraState.IsStopped;
        private List<Mat> _gtTransformations;
        private int _gtIndex = 0;
        private readonly int CameraTimePerFrame = 100; //ms

        private bool _addPoints = false;
        private float _fps;

        //private float camera_fx = 458.654f, camera_fy = 457.296f, camera_cx = 367.215f, camera_cy = 248.375f; // euroC

        private float camera_fx = 718.856f, camera_fy = 718.856f, camera_cx = 607.1928f, camera_cy = 185.2157f; // kitti

        //3D rendering camera pose
        private Render3D _render3D;

        // key points for drawing
        KeyPoint[] _keyPointsPrevious;
        KeyPoint[] _keyPointsCurrent;

        public RoLabsSlamForm()
        {
            InitializeComponent();
            InitializeVideoCapture();
        }

        private void glControl_Load(object? sender, EventArgs e)
        {
            _render3D = new Render3D(glControl);

            _render3D.glControl_Load(sender, e);

            radioGT.Checked = false;
            radioReal.Checked = true;

            textBoxGT.Text = @"D:\\work\\125.MyRobotics\\dataset\\00\\poses\\00.txt";
        }

        private void InitializeVideoCapture()
        {
            _frame = new Mat();
            _timer = new System.Windows.Forms.Timer
            {
                Interval = CameraTimePerFrame
            };
            _timer.Tick += Timer_Tick;

            _rolabsSlamWrapper = new RolabsSlamSharpWrapper();
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            if (_cameraState == CameraState.IsPaused) { return; }

            if (!radioGT.Checked)
            {
                if (_videoCapture.Read(_frame))
                {
                    // Convert the Mat to Bitmap on the background thread
                    Bitmap newBitmap = BitmapConverter.ToBitmap(_frame);

                    // Use Invoke to update the UI on the main thread
                    pictureBoxRaw.Invoke(new Action(() =>
                    {
                        pictureBoxRaw.Image?.Dispose();
                        pictureBoxRaw.Image = newBitmap;
                    }));

                    // Create a stopwatch instance
                    Stopwatch stopwatch = new Stopwatch();

                    // Start the stopwatch before the first method call
                    stopwatch.Start();

                    _rolabsSlamWrapper.GrabImage(_frame);
                    _rolabsSlamWrapper.Track();

                    // Record the elapsed time for GrabImage
                    stopwatch.Stop();

                    float duration = stopwatch.ElapsedMilliseconds;
                    Console.WriteLine($"Tracking Time: {duration} ms");

                    _fps = 1 / (duration * 0.001f);

                    (_keyPointsCurrent, _keyPointsPrevious) = _rolabsSlamWrapper.GetDebugKeyPoints();
                    Mat pose = _rolabsSlamWrapper.GetPose();

                    Matrix4 matrix4 = pose.ToMatrix4();

                    Console.WriteLine($"Camera Pose {matrix4} ================>  frame count: {_gtIndex}");

                    Point3f[] mapPoints = _rolabsSlamWrapper.GetMapPoints();

                    Mat debugImg = _frame.Clone();

                    if (_keyPointsPrevious.Length > 0)
                    {
                        for (int i = 0; i < _keyPointsCurrent.Length; i++)
                        {
                            var currentPoint = _keyPointsCurrent[i];
                            var previousPoint = _keyPointsPrevious[i];

                            // Draw an arrow between the matched keypoints
                            Cv2.ArrowedLine(debugImg, new OpenCvSharp.Point((int)previousPoint.Pt.X, (int)previousPoint.Pt.Y),
                                                      new OpenCvSharp.Point((int)currentPoint.Pt.X, (int)currentPoint.Pt.Y),
                                                      new Scalar(0, 255, 0), 2, LineTypes.AntiAlias, 0, 0.2);
                        }
                    }
                    else
                    {
                        // Draw circles at each keypoint
                        foreach (var keypoint in _keyPointsCurrent)
                        {
                            // Draw a circle at each keypoint position
                            Cv2.Circle(debugImg, (OpenCvSharp.Point)keypoint.Pt, 3, Scalar.Green, 2);
                        }
                    }

                    // Convert the Mat to Bitmap on the background thread
                    Bitmap processBitmap = BitmapConverter.ToBitmap(debugImg);

                    // Use Invoke to update the UI on the main thread
                    pictureBoxProcess.Invoke(new Action(() =>
                    {
                        pictureBoxProcess.Image?.Dispose();
                        pictureBoxProcess.Image = processBitmap;
                    }));

                    // render camera poses
                    matrix4.Invert();
                    matrix4.M14 *= VisualizedScaled;
                    matrix4.M24 *= VisualizedScaled;
                    matrix4.M34 *= VisualizedScaled;
                    matrix4.Transpose();
                    _render3D.AddPyramidTransformation(matrix4);

                    if (_addPoints)
                    {
                        IEnumerable<Vector3> glPoints = mapPoints.Select(p => new Vector3(p.X * VisualizedScaled, p.Y * VisualizedScaled, p.Z * VisualizedScaled));
                        Color4[] colors = Enumerable.Repeat(Color4.Green, mapPoints.Length).ToArray();
                        _render3D.ClearPoints();
                        _render3D.AddPoints(glPoints, colors);
                    }
                    _gtIndex++;
                }
                else
                {
                    _timer.Stop();
                    _videoCapture.Release();
                }

                frameNo.Invoke(new Action(() => frameNo.Text = $"Frame No: {_gtIndex}"));
                labelFps.Invoke(new Action(() => labelFps.Text = $"Fps: {_fps}"));

            }
            else
            {
                if (_videoCapture.Read(_frame))
                {
                    // Convert the Mat to Bitmap on the background thread
                    Bitmap newBitmap = BitmapConverter.ToBitmap(_frame);

                    // Use Invoke to update the UI on the main thread
                    pictureBoxRaw.Invoke(new Action(() =>
                    {
                        pictureBoxRaw.Image?.Dispose();
                        pictureBoxRaw.Image = newBitmap;
                    }));
                }
                else
                {
                    _timer.Stop();
                    _videoCapture.Release();
                }

                Matrix4 matrix4 = _gtTransformations[_gtIndex].ToMatrix4();

                matrix4.Transpose();

                _render3D.AddPyramidTransformation(matrix4);
                _gtIndex++;
            }

            //_cameraState = CameraState.IsPaused;
        }

        private void startButton_Click(object sender, EventArgs e)
        {
            if (_cameraState == CameraState.IsStopped)
            {
                _cameraState = CameraState.IsStarted;
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string projectDir = Directory.GetParent(baseDir).Parent.Parent.FullName;
                string videoPathMerged = textBoxVideoPath.Text == "" ? projectDir + videoPath : textBoxVideoPath.Text;
                _videoCapture = new VideoCapture(videoPathMerged);
                _timer.Start();
                if (radioGT.Checked == true)
                {
                    try
                    {
                        _gtTransformations = GTParser.Parse(textBoxGT.Text);
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message);
                    }

                }
                else
                {
                    _rolabsSlamWrapper.SetCameraIntrinsics(camera_fx, camera_fy, camera_cx, camera_cy);
                    _rolabsSlamWrapper.Start();
                }

                radioGT.Enabled = false;
                radioReal.Enabled = false;
                textBoxGT.Enabled = false;
            }
            else if (_cameraState == CameraState.IsPaused)
            {
                _cameraState = CameraState.IsStarted;
            }
        }

        private void stopButton_Click(object sender, EventArgs e)
        {
            _gtIndex = 0;
            _cameraState = CameraState.IsStopped;
            _videoCapture.Release();
            _timer.Stop();
            _rolabsSlamWrapper.Stop();
            radioGT.Enabled = true;
            radioReal.Enabled = true;
            textBoxGT.Enabled = true;
        }

        private void pauseButton_Click(object sender, EventArgs e)
        {
            _cameraState = CameraState.IsPaused;
        }

        private void checkBoxPointCloud_CheckedChanged(object sender, EventArgs e)
        {
            // Cast sender to CheckBox to access its properties
            System.Windows.Forms.CheckBox checkBox = sender as System.Windows.Forms.CheckBox;

            if (checkBox != null)
            {
                if (checkBox.Checked)
                {
                    _addPoints = true;
                }
                else
                {
                    _addPoints = false;
                    _render3D.ClearPoints();
                }
            }
        }
    }
}
