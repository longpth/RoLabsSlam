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

namespace RoLabsSlam.Test
{
    enum CameraState
    {
        IsStopped = 1, IsStarted = 2, IsPaused = 3,
    }
    public partial class RoLabsSlamForm : Form
    {
        private VideoCapture _videoCapture;
        private Mat _frame;
        private System.Windows.Forms.Timer _timer;
        private RolabsSlamSharpWrapper _rolabsSlamWrapper;
        private CameraState _cameraState = CameraState.IsStopped;
        private List<Mat> _gtTransformations;
        private int _gtIndex = 0;
        private readonly int CameraTimePerFrame = 100; //ms

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

            this.KeyPreview = true;  // Enable KeyPreview to capture key events at the form level
            this.KeyDown += new KeyEventHandler(RoLabsSlamForm_KeyDown);  // Attach the KeyDown event handler
            this.MouseDown += RoLabsSlamForm_MouseDown;
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

                    Console.WriteLine($"Tracking Time: {stopwatch.ElapsedMilliseconds} ms");

                    (_keyPointsCurrent , _keyPointsPrevious) = _rolabsSlamWrapper.GetDebugKeyPoints();
                    Mat pose = _rolabsSlamWrapper.GetPose();

                    Matrix4 matrix4 = pose.ToMatrix4();

                    Console.WriteLine($"Camera Pose {matrix4}");

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

                }
                else
                {
                    _timer.Stop();
                    _videoCapture.Release();
                }
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
        }

        private void startButton_Click(object sender, EventArgs e)
        {
            if (_cameraState == CameraState.IsStopped)
            {
                _cameraState = CameraState.IsStarted;
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string projectDir = Directory.GetParent(baseDir).Parent.Parent.FullName;
                //string videoPath = projectDir + @"\..\..\RoLabs\Resources\Raw\slam\video\euroc_V2_01_easy.mp4";
                string videoPath = projectDir + @"\..\..\..\..\..\dataset\kitti_sequence_00.mp4";
                _videoCapture = new VideoCapture(videoPath);
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
        }

        private void pauseButton_Click(object sender, EventArgs e)
        {
            _cameraState = CameraState.IsPaused;
        }

        private void RoLabsSlamForm_KeyDown(object sender, KeyEventArgs e)
        {
            // Handle key press without giving focus to the TextBox
            if (e.KeyCode == Keys.Up || 
                e.KeyCode == Keys.Down || 
                e.KeyCode == Keys.Left || 
                e.KeyCode == Keys.Right || 
                e.KeyCode == Keys.PageUp || 
                e.KeyCode == Keys.PageDown)
            {
                // Prevent the TextBox from getting focus by handling the keys
                e.Handled = true;
                e.SuppressKeyPress = true;

                // Call your camera update method
                HandleCameraMovement(e.KeyCode);
            }
        }

        private void HandleCameraMovement(Keys key)
        {
            float delta = 1.0f; // Your delta value

            switch (key)
            {
                case Keys.Up:
                    _render3D.UpdateCameraPosition(delta, CameraDirection.Forward);
                    break;
                case Keys.Down:
                    _render3D.UpdateCameraPosition(delta, CameraDirection.Backward);
                    break;
                case Keys.Left:
                    _render3D.UpdateCameraPosition(delta, CameraDirection.Left);
                    break;
                case Keys.Right:
                    _render3D.UpdateCameraPosition(delta, CameraDirection.Right);
                    break;
                case Keys.PageUp:
                    _render3D.UpdateCameraPosition(delta, CameraDirection.Up);
                    break;
                case Keys.PageDown:
                    _render3D.UpdateCameraPosition(delta, CameraDirection.Down);
                    break;
            }
        }

        private void RoLabsSlamForm_MouseDown(object sender, MouseEventArgs e)
        {
            // Set focus to the form itself or another control like a button
            glControl.Focus();
        }

    }
}
