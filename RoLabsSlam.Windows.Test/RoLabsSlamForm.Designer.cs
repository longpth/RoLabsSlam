

namespace RoLabsSlam.Test
{
    partial class RoLabsSlamForm
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components?.Dispose();
                _frame?.Dispose();
                _videoCapture?.Dispose();
                _timer?.Dispose();
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            pictureBoxRaw = new PictureBox();
            pictureBoxProcess = new PictureBox();
            startButton = new System.Windows.Forms.Button();
            stopButton = new System.Windows.Forms.Button();
            glControl = new OpenTK.WinForms.GLControl();
            textBoxGT = new TextBox();
            label1 = new System.Windows.Forms.Label();
            radioGT = new System.Windows.Forms.RadioButton();
            radioReal = new System.Windows.Forms.RadioButton();
            pauseButton = new System.Windows.Forms.Button();
            groupBox1 = new GroupBox();
            groupBox2 = new GroupBox();
            ((System.ComponentModel.ISupportInitialize)pictureBoxRaw).BeginInit();
            ((System.ComponentModel.ISupportInitialize)pictureBoxProcess).BeginInit();
            groupBox1.SuspendLayout();
            groupBox2.SuspendLayout();
            SuspendLayout();
            // 
            // pictureBoxRaw
            // 
            pictureBoxRaw.Location = new System.Drawing.Point(0, 14);
            pictureBoxRaw.Name = "pictureBoxRaw";
            pictureBoxRaw.Size = new System.Drawing.Size(1018, 405);
            pictureBoxRaw.TabIndex = 0;
            pictureBoxRaw.TabStop = false;
            // 
            // pictureBoxProcess
            // 
            pictureBoxProcess.Location = new System.Drawing.Point(0, 422);
            pictureBoxProcess.Name = "pictureBoxProcess";
            pictureBoxProcess.Size = new System.Drawing.Size(1018, 429);
            pictureBoxProcess.TabIndex = 1;
            pictureBoxProcess.TabStop = false;
            // 
            // startButton
            // 
            startButton.Location = new System.Drawing.Point(6, 14);
            startButton.Name = "startButton";
            startButton.Size = new System.Drawing.Size(94, 29);
            startButton.TabIndex = 2;
            startButton.Text = "Start";
            startButton.UseVisualStyleBackColor = true;
            startButton.Click += startButton_Click;
            // 
            // stopButton
            // 
            stopButton.Location = new System.Drawing.Point(6, 46);
            stopButton.Name = "stopButton";
            stopButton.Size = new System.Drawing.Size(94, 29);
            stopButton.TabIndex = 3;
            stopButton.Text = "Stop";
            stopButton.UseVisualStyleBackColor = true;
            stopButton.Click += stopButton_Click;
            // 
            // glControl
            // 
            glControl.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            glControl.API = OpenTK.Windowing.Common.ContextAPI.OpenGL;
            glControl.APIVersion = new Version(3, 3, 0, 0);
            glControl.Flags = OpenTK.Windowing.Common.ContextFlags.Debug;
            glControl.IsEventDriven = true;
            glControl.Location = new System.Drawing.Point(1024, 14);
            glControl.Margin = new Padding(3, 4, 3, 4);
            glControl.Name = "glControl";
            glControl.Profile = OpenTK.Windowing.Common.ContextProfile.Core;
            glControl.Size = new System.Drawing.Size(948, 837);
            glControl.TabIndex = 4;
            glControl.Text = "glControl1";
            glControl.Load += glControl_Load;
            // 
            // textBoxGT
            // 
            textBoxGT.Location = new System.Drawing.Point(251, 18);
            textBoxGT.Name = "textBoxGT";
            textBoxGT.Size = new System.Drawing.Size(757, 27);
            textBoxGT.TabIndex = 5;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.Location = new System.Drawing.Point(531, 876);
            label1.Name = "label1";
            label1.Size = new System.Drawing.Size(95, 20);
            label1.TabIndex = 6;
            label1.Text = "Ground Truth";
            // 
            // radioGT
            // 
            radioGT.AutoSize = true;
            radioGT.Location = new System.Drawing.Point(129, 19);
            radioGT.Name = "radioGT";
            radioGT.Size = new System.Drawing.Size(116, 24);
            radioGT.TabIndex = 7;
            radioGT.TabStop = true;
            radioGT.Text = "Ground Truth";
            radioGT.UseVisualStyleBackColor = true;
            // 
            // radioReal
            // 
            radioReal.AutoSize = true;
            radioReal.Location = new System.Drawing.Point(129, 49);
            radioReal.Name = "radioReal";
            radioReal.Size = new System.Drawing.Size(96, 24);
            radioReal.TabIndex = 8;
            radioReal.TabStop = true;
            radioReal.Text = "Real Slam";
            radioReal.UseVisualStyleBackColor = true;
            // 
            // pauseButton
            // 
            pauseButton.Location = new System.Drawing.Point(6, 78);
            pauseButton.Name = "pauseButton";
            pauseButton.Size = new System.Drawing.Size(94, 29);
            pauseButton.TabIndex = 9;
            pauseButton.Text = "Pause";
            pauseButton.UseVisualStyleBackColor = true;
            pauseButton.Click += pauseButton_Click;
            // 
            // groupBox1
            // 
            groupBox1.Controls.Add(pictureBoxProcess);
            groupBox1.Controls.Add(pictureBoxRaw);
            groupBox1.Controls.Add(glControl);
            groupBox1.Location = new System.Drawing.Point(6, -3);
            groupBox1.Name = "groupBox1";
            groupBox1.Size = new System.Drawing.Size(1982, 858);
            groupBox1.TabIndex = 10;
            groupBox1.TabStop = false;
            // 
            // groupBox2
            // 
            groupBox2.Controls.Add(startButton);
            groupBox2.Controls.Add(stopButton);
            groupBox2.Controls.Add(pauseButton);
            groupBox2.Controls.Add(textBoxGT);
            groupBox2.Controls.Add(radioGT);
            groupBox2.Controls.Add(radioReal);
            groupBox2.Location = new System.Drawing.Point(6, 857);
            groupBox2.Name = "groupBox2";
            groupBox2.Size = new System.Drawing.Size(1982, 109);
            groupBox2.TabIndex = 11;
            groupBox2.TabStop = false;
            // 
            // RoLabsSlamForm
            // 
            AutoScaleDimensions = new System.Drawing.SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new System.Drawing.Size(1990, 978);
            Controls.Add(groupBox2);
            Controls.Add(groupBox1);
            Controls.Add(label1);
            Name = "RoLabsSlamForm";
            Text = "Form1";
            ((System.ComponentModel.ISupportInitialize)pictureBoxRaw).EndInit();
            ((System.ComponentModel.ISupportInitialize)pictureBoxProcess).EndInit();
            groupBox1.ResumeLayout(false);
            groupBox2.ResumeLayout(false);
            groupBox2.PerformLayout();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private PictureBox pictureBoxRaw;
        private PictureBox pictureBoxProcess;
        private System.Windows.Forms.Button startButton;
        private System.Windows.Forms.Button stopButton;
        private OpenTK.WinForms.GLControl glControl;
        private TextBox textBoxGT;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.RadioButton radioGT;
        private System.Windows.Forms.RadioButton radioReal;
        private System.Windows.Forms.Button pauseButton;
        private GroupBox groupBox1;
        private GroupBox groupBox2;
    }
}
