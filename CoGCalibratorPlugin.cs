using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Controls;
using MissionPlanner.Controls.BackstageView;

namespace MissionPlanner.CoGCalibrator
{
    // Mission Planner Plugin for estimating Center of Gravity (CoG) relative to the IMU
    // Adds a calibration page to the SETUP view under Optional Hardware.
    public class CoGCalibratorPlugin : MissionPlanner.Plugin.Plugin
    {
        public override string Name { get { return "Virtual Plumb Bob CoG Calibrator"; } }
        public override string Version { get { return "0.1.0"; } }
        public override string Author { get { return "Yuri"; } }

        public override bool Init()
        {
            return true;
        }

        public override bool Loaded()
        {
            Application.Idle += OnApplicationIdle;
            return true;
        }

        public override bool Exit()
        {
            Application.Idle -= OnApplicationIdle;
            return true;
        }

        private void OnApplicationIdle(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.instance == null) return;

                GCSViews.InitialSetup setupView = FindInitialSetupControl(MainV2.instance);
                if (setupView != null)
                {
                    System.Reflection.FieldInfo field = typeof(GCSViews.InitialSetup).GetField("backstageView", 
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    
                    if (field == null) return;

                    MissionPlanner.Controls.BackstageView.BackstageView backstageView = 
                        field.GetValue(setupView) as MissionPlanner.Controls.BackstageView.BackstageView;
                    
                    if (backstageView != null)
                    {
                        bool alreadyAdded = false;
                        foreach (BackstageViewPage page in backstageView.Pages)
                        {
                            if (page.LinkText == "CoG Estimator")
                            {
                                alreadyAdded = true;
                                break;
                            }
                        }

                        if (!alreadyAdded)
                        {
                            BackstageViewPage optPage = null;
                            foreach (BackstageViewPage page in backstageView.Pages)
                            {
                                if (page.LinkText == "Optional Hardware" || page.LinkText.Contains("Optional"))
                                {
                                    optPage = page;
                                    break;
                                }
                            }
                            backstageView.AddPage(typeof(CoGCalibratorControl), "CoG Estimator", optPage, false);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("CoG Plugin Idle hook error: " + ex.Message);
            }
        }

        private static GCSViews.InitialSetup FindInitialSetupControl(Control parent)
        {
            if (parent is GCSViews.InitialSetup)
                return (GCSViews.InitialSetup)parent;

            foreach (Control child in parent.Controls)
            {
                GCSViews.InitialSetup found = FindInitialSetupControl(child);
                if (found != null)
                    return found;
            }
            return null;
        }
    }

    public class CoGCalibratorControl : UserControl
    {
        public class Measurement
        {
            public int Index { get; set; }
            public double HangX { get; set; } // meters
            public double HangY { get; set; } // meters
            public double HangZ { get; set; } // meters
            public double AccelX { get; set; } // mG or scaled
            public double AccelY { get; set; }
            public double AccelZ { get; set; }
            public double[] GravityDir { get; set; } // Normalized gravity unit vector in body frame
            public double Residual { get; set; } // mm
        }

        private class ParamVal
        {
            public string Name;
            public float Value;
            public ParamVal(string name, float val)
            {
                Name = name;
                Value = val;
            }
        }

        private List<Measurement> _measurements = new List<Measurement>();
        private int _measurementCounter = 0;

        // Telemetry Sampling state variables
        private bool _isSampling = false;
        private double _sampleSumX = 0;
        private double _sampleSumY = 0;
        private double _sampleSumZ = 0;
        private int _sampleCount = 0;
        private readonly object _lockObj = new object();
        private DateTime _sampleStartTime;
        private double _sampleDurationSec = 3.0;
        
        // Fallback sampling timer state
        private List<double[]> _fallbackSamples = new List<double[]>();
        private Timer _samplingTimer;
        private Timer _uiUpdateTimer;
        private Timer _connectionTimer;

        // UI Controls
        private Panel _headerPanel;
        private Label _titleLabel;
        private Label _subtitleLabel;

        private GroupBox _inputGroup;
        private NumericUpDown _numX;
        private NumericUpDown _numY;
        private NumericUpDown _numZ;
        private Button _btnMeasure;
        private ProgressBar _progressBar;
        private Label _lblStatus;

        private GroupBox _listGroup;
        private DataGridView _dataGridView;
        private Button _btnDelete;
        private Button _btnClear;

        private GroupBox _resultsGroup;
        private Label _lblCogResult;
        private Label _lblParamResult;
        private CheckBox _chkImu1;
        private CheckBox _chkImu2;
        private CheckBox _chkImu3;
        private Button _btnWriteParams;

        public CoGCalibratorControl()
        {
            InitializeComponent();
            SetupFormLayout();
            SetupTelemetrySubscription();
        }

        private void InitializeComponent()
        {
            this.BackColor = Color.FromArgb(24, 28, 36);
            this.ForeColor = Color.FromArgb(220, 224, 230);
            this.Font = new Font("Segoe UI", 9F);
        }

        private void SetupFormLayout()
        {
            // 1. Header Panel
            _headerPanel = new Panel();
            _headerPanel.Dock = DockStyle.Top;
            _headerPanel.Height = 90;
            _headerPanel.BackColor = Color.FromArgb(18, 20, 28);
            _headerPanel.Paint += HeaderPanel_Paint;

            _titleLabel = new Label();
            _titleLabel.Text = "Virtual Plumb Bob IMU Offset Calibrator";
            _titleLabel.Font = new Font("Segoe UI Semibold", 16F, FontStyle.Bold);
            _titleLabel.ForeColor = Color.White;
            _titleLabel.Location = new Point(20, 15);
            _titleLabel.AutoSize = true;

            _subtitleLabel = new Label();
            _subtitleLabel.Text = "Determine 2+ hang points and measure their offsets relative to the IMU. Hang the vehicle from each point, running at least one static measurement for each.";
            _subtitleLabel.Font = new Font("Segoe UI", 9F, FontStyle.Italic);
            _subtitleLabel.ForeColor = Color.FromArgb(160, 168, 180);
            _subtitleLabel.Location = new Point(20, 52);
            _subtitleLabel.AutoSize = true;

            _headerPanel.Controls.Add(_titleLabel);
            _headerPanel.Controls.Add(_subtitleLabel);
            this.Controls.Add(_headerPanel);

            // Main layout using TableLayoutPanel
            TableLayoutPanel mainLayout = new TableLayoutPanel();
            mainLayout.Dock = DockStyle.Fill;
            mainLayout.ColumnCount = 2;
            mainLayout.RowCount = 2;
            mainLayout.Padding = new Padding(15, 105, 15, 15);
            mainLayout.BackColor = Color.FromArgb(24, 28, 36);
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 45F));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55F));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 65F));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 35F));
            this.Controls.Add(mainLayout);

            // 2. Input Group (Top-Left)
            _inputGroup = new GroupBox();
            _inputGroup.Text = "1. Add Hang Point Measurement";
            _inputGroup.Dock = DockStyle.Fill;
            _inputGroup.ForeColor = Color.White;
            _inputGroup.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            _inputGroup.Padding = new Padding(10);

            TableLayoutPanel inputLayout = new TableLayoutPanel();
            inputLayout.Dock = DockStyle.Fill;
            inputLayout.ColumnCount = 2;
            inputLayout.RowCount = 6;
            inputLayout.Font = new Font("Segoe UI", 9F);
            inputLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40F));
            inputLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 60F));

            string[] axes = { "X Offset (Forward) [mm]:", "Y Offset (Right) [mm]:", "Z Offset (Down) [mm]:" };
            NumericUpDown[] nudControls = new NumericUpDown[3];
            for (int i = 0; i < 3; i++)
            {
                Label lbl = new Label();
                lbl.Text = axes[i];
                lbl.TextAlign = ContentAlignment.MiddleLeft;
                lbl.Dock = DockStyle.Fill;
                lbl.ForeColor = Color.FromArgb(200, 204, 210);

                NumericUpDown nud = new NumericUpDown();
                nud.Minimum = -2000;
                nud.Maximum = 2000;
                nud.DecimalPlaces = 0;
                nud.Value = 0;
                nud.Dock = DockStyle.Fill;
                nud.BackColor = Color.FromArgb(38, 44, 56);
                nud.ForeColor = Color.White;
                nud.BorderStyle = BorderStyle.FixedSingle;

                inputLayout.Controls.Add(lbl, 0, i);
                inputLayout.Controls.Add(nud, 1, i);
                nudControls[i] = nud;
            }
            _numX = nudControls[0];
            _numY = nudControls[1];
            _numZ = nudControls[2];

            Label lblTip = new Label();
            lblTip.Text = "Measure the x-y-z distance from the physical IMU center to the point where the hang-string attaches to the drone frame. Note: Z-up measurements are negative.";
            lblTip.Font = new Font("Segoe UI", 8F, FontStyle.Italic);
            lblTip.ForeColor = Color.FromArgb(160, 168, 180);
            lblTip.Dock = DockStyle.Fill;
            lblTip.Height = 45;
            inputLayout.Controls.Add(lblTip, 0, 3);
            inputLayout.SetColumnSpan(lblTip, 2);

            _btnMeasure = new Button();
            _btnMeasure.Text = "Start 3s Static Measurement";
            _btnMeasure.Dock = DockStyle.Fill;
            _btnMeasure.Height = 35;
            _btnMeasure.FlatStyle = FlatStyle.Flat;
            _btnMeasure.BackColor = Color.FromArgb(45, 140, 90);
            _btnMeasure.ForeColor = Color.White;
            _btnMeasure.Font = new Font("Segoe UI Semibold", 9.5F, FontStyle.Bold);
            _btnMeasure.FlatAppearance.BorderSize = 0;
            _btnMeasure.Click += BtnMeasure_Click;
            inputLayout.Controls.Add(_btnMeasure, 0, 4);
            inputLayout.SetColumnSpan(_btnMeasure, 2);

            FlowLayoutPanel statusLayout = new FlowLayoutPanel();
            statusLayout.Dock = DockStyle.Fill;
            statusLayout.FlowDirection = FlowDirection.TopDown;
            statusLayout.WrapContents = false;

            _progressBar = new ProgressBar();
            _progressBar.Width = 280;
            _progressBar.Height = 12;
            _progressBar.Style = ProgressBarStyle.Blocks;
            _progressBar.Visible = false;

            _lblStatus = new Label();
            _lblStatus.Text = "Ready for measurement.";
            _lblStatus.AutoSize = true;
            _lblStatus.ForeColor = Color.FromArgb(180, 184, 190);
            _lblStatus.Font = new Font("Segoe UI", 9F);

            statusLayout.Controls.Add(_progressBar);
            statusLayout.Controls.Add(_lblStatus);
            inputLayout.Controls.Add(statusLayout, 0, 5);
            inputLayout.SetColumnSpan(statusLayout, 2);

            _inputGroup.Controls.Add(inputLayout);
            mainLayout.Controls.Add(_inputGroup, 0, 0);

            // 3. List Group (Top-Right)
            _listGroup = new GroupBox();
            _listGroup.Text = "2. Measurement Results";
            _listGroup.Dock = DockStyle.Fill;
            _listGroup.ForeColor = Color.White;
            _listGroup.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            _listGroup.Padding = new Padding(10);

            TableLayoutPanel listLayout = new TableLayoutPanel();
            listLayout.Dock = DockStyle.Fill;
            listLayout.ColumnCount = 2;
            listLayout.RowCount = 2;
            listLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            listLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            listLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            listLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 45F));

            _dataGridView = new DataGridView();
            _dataGridView.Dock = DockStyle.Fill;
            _dataGridView.AllowUserToAddRows = false;
            _dataGridView.AllowUserToDeleteRows = false;
            _dataGridView.ReadOnly = true;
            _dataGridView.SelectionMode = DataGridViewSelectionMode.FullRowSelect;
            _dataGridView.MultiSelect = false;
            _dataGridView.BackgroundColor = Color.FromArgb(30, 36, 48);
            _dataGridView.ForeColor = Color.White;
            _dataGridView.BorderStyle = BorderStyle.None;
            _dataGridView.GridColor = Color.FromArgb(50, 58, 72);
            _dataGridView.RowHeadersVisible = false;
            _dataGridView.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
            _dataGridView.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(18, 20, 28);
            _dataGridView.ColumnHeadersDefaultCellStyle.ForeColor = Color.White;
            _dataGridView.EnableHeadersVisualStyles = false;
            _dataGridView.DefaultCellStyle.BackColor = Color.FromArgb(30, 36, 48);
            _dataGridView.DefaultCellStyle.SelectionBackColor = Color.FromArgb(50, 100, 180);
            _dataGridView.DefaultCellStyle.SelectionForeColor = Color.White;

            _dataGridView.Columns.Add("Index", "#");
            _dataGridView.Columns.Add("HangPoint", "Hang Point (X,Y,Z) [mm]");
            _dataGridView.Columns.Add("Gravity", "Gravity Vector (X,Y,Z)");
            _dataGridView.Columns.Add("Residual", "Residual");

            _dataGridView.Columns[0].AutoSizeMode = DataGridViewAutoSizeColumnMode.None;
            _dataGridView.Columns[0].Width = 40;
            _dataGridView.Columns[1].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;
            _dataGridView.Columns[2].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;
            _dataGridView.Columns[3].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;

            listLayout.Controls.Add(_dataGridView, 0, 0);
            listLayout.SetColumnSpan(_dataGridView, 2);

            _btnDelete = new Button();
            _btnDelete.Text = "Delete Selected";
            _btnDelete.Anchor = AnchorStyles.Left;
            _btnDelete.Width = 130;
            _btnDelete.Height = 35;
            _btnDelete.FlatStyle = FlatStyle.Flat;
            _btnDelete.BackColor = Color.FromArgb(150, 50, 50);
            _btnDelete.ForeColor = Color.White;
            _btnDelete.Font = new Font("Segoe UI", 9F);
            _btnDelete.FlatAppearance.BorderSize = 0;
            _btnDelete.Click += BtnDelete_Click;
            listLayout.Controls.Add(_btnDelete, 0, 1);

            _btnClear = new Button();
            _btnClear.Text = "Clear All";
            _btnClear.Anchor = AnchorStyles.Right;
            _btnClear.Width = 100;
            _btnClear.Height = 35;
            _btnClear.FlatStyle = FlatStyle.Flat;
            _btnClear.BackColor = Color.FromArgb(70, 78, 92);
            _btnClear.ForeColor = Color.White;
            _btnClear.Font = new Font("Segoe UI", 9F);
            _btnClear.FlatAppearance.BorderSize = 0;
            _btnClear.Click += BtnClear_Click;
            listLayout.Controls.Add(_btnClear, 1, 1);

            _listGroup.Controls.Add(listLayout);
            mainLayout.Controls.Add(_listGroup, 1, 0);

            // 4. Results Group (Bottom - spans both columns)
            _resultsGroup = new GroupBox();
            _resultsGroup.Text = "3. Calibration Results";
            _resultsGroup.Dock = DockStyle.Fill;
            _resultsGroup.ForeColor = Color.White;
            _resultsGroup.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            _resultsGroup.Padding = new Padding(15);

            TableLayoutPanel resultsLayout = new TableLayoutPanel();
            resultsLayout.Dock = DockStyle.Fill;
            resultsLayout.ColumnCount = 4;
            resultsLayout.RowCount = 1;
            resultsLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 32F));
            resultsLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 32F));
            resultsLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 18F));
            resultsLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 18F));

            _lblCogResult = new Label();
            _lblCogResult.Text = "Center of Gravity (CoG) relative to IMU:\n- X (Forward): ---\n- Y (Right): ---\n- Z (Down): ---";
            _lblCogResult.Font = new Font("Segoe UI", 9.5F);
            _lblCogResult.ForeColor = Color.FromArgb(220, 224, 230);
            _lblCogResult.Dock = DockStyle.Fill;
            _lblCogResult.TextAlign = ContentAlignment.TopLeft;
            _lblCogResult.Margin = new Padding(5, 10, 5, 5);
            resultsLayout.Controls.Add(_lblCogResult, 0, 0);

            _lblParamResult = new Label();
            _lblParamResult.Text = "Derived INS_POS offsets:\n- INS_POS_X: ---\n- INS_POS_Y: ---\n- INS_POS_Z: ---";
            _lblParamResult.Font = new Font("Segoe UI", 9.5F);
            _lblParamResult.ForeColor = Color.FromArgb(220, 224, 230);
            _lblParamResult.Dock = DockStyle.Fill;
            _lblParamResult.TextAlign = ContentAlignment.TopLeft;
            _lblParamResult.Margin = new Padding(5, 10, 5, 5);
            resultsLayout.Controls.Add(_lblParamResult, 1, 0);

            FlowLayoutPanel checkPanel = new FlowLayoutPanel();
            checkPanel.Dock = DockStyle.Fill;
            checkPanel.FlowDirection = FlowDirection.TopDown;
            checkPanel.Margin = new Padding(5, 5, 5, 5);
            checkPanel.Padding = new Padding(0, 5, 0, 0);

            Label lblTarget = new Label();
            lblTarget.Text = "Target IMUs:";
            lblTarget.Font = new Font("Segoe UI Semibold", 9F, FontStyle.Bold);
            lblTarget.ForeColor = Color.White;
            lblTarget.Height = 20;
            lblTarget.Width = 120;
            lblTarget.Margin = new Padding(0);

            _chkImu1 = new CheckBox();
            _chkImu1.Text = "IMU 1 (INS_POS1)";
            _chkImu1.Checked = false;
            _chkImu1.Enabled = false;
            _chkImu1.AutoSize = true;
            _chkImu1.Font = new Font("Segoe UI", 8F);
            _chkImu1.ForeColor = Color.FromArgb(200, 204, 210);
            _chkImu1.Margin = new Padding(0, 3, 0, 0);

            _chkImu2 = new CheckBox();
            _chkImu2.Text = "IMU 2 (INS_POS2)";
            _chkImu2.Checked = false;
            _chkImu2.Enabled = false;
            _chkImu2.AutoSize = true;
            _chkImu2.Font = new Font("Segoe UI", 8F);
            _chkImu2.ForeColor = Color.FromArgb(200, 204, 210);
            _chkImu2.Margin = new Padding(0, 3, 0, 0);

            _chkImu3 = new CheckBox();
            _chkImu3.Text = "IMU 3 (INS_POS3)";
            _chkImu3.Checked = false;
            _chkImu3.Enabled = false;
            _chkImu3.AutoSize = true;
            _chkImu3.Font = new Font("Segoe UI", 8F);
            _chkImu3.ForeColor = Color.FromArgb(200, 204, 210);
            _chkImu3.Margin = new Padding(0, 3, 0, 0);

            checkPanel.Controls.Add(lblTarget);
            checkPanel.Controls.Add(_chkImu1);
            checkPanel.Controls.Add(_chkImu2);
            checkPanel.Controls.Add(_chkImu3);
            resultsLayout.Controls.Add(checkPanel, 2, 0);

            _btnWriteParams = new Button();
            _btnWriteParams.Text = "Write INS_POS\nParameters";
            _btnWriteParams.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            _btnWriteParams.Height = 35;
            _btnWriteParams.Margin = new Padding(5, 10, 5, 5);
            _btnWriteParams.FlatStyle = FlatStyle.Flat;
            _btnWriteParams.BackColor = Color.FromArgb(50, 100, 180);
            _btnWriteParams.ForeColor = Color.White;
            _btnWriteParams.Font = new Font("Segoe UI Semibold", 9.5F, FontStyle.Bold);
            _btnWriteParams.Enabled = false;
            _btnWriteParams.FlatAppearance.BorderSize = 0;
            _btnWriteParams.Click += BtnWriteParams_Click;
            resultsLayout.Controls.Add(_btnWriteParams, 3, 0);

            _resultsGroup.Controls.Add(resultsLayout);
            mainLayout.Controls.Add(_resultsGroup, 0, 1);
            mainLayout.SetColumnSpan(_resultsGroup, 2);
        }

        private void HeaderPanel_Paint(object sender, PaintEventArgs e)
        {
            using (Pen pen = new Pen(Color.FromArgb(50, 58, 72), 1))
            {
                e.Graphics.DrawLine(pen, 0, _headerPanel.Height - 1, _headerPanel.Width, _headerPanel.Height - 1);
            }
        }

        private void SetupTelemetrySubscription()
        {
            // Subscribe to the MAVLink packet callback
            MainV2.comPort.OnPacketReceived += ComPort_OnPacketReceived;
            
            // Set up fallback and UI timers
            _samplingTimer = new Timer();
            _samplingTimer.Interval = 20; // 50 Hz
            _samplingTimer.Tick += SamplingTimer_Tick;

            _uiUpdateTimer = new Timer();
            _uiUpdateTimer.Interval = 100; // 10 Hz for progress bar / count labels
            _uiUpdateTimer.Tick += UiUpdateTimer_Tick;

            _connectionTimer = new Timer();
            _connectionTimer.Interval = 1000; // 1 Hz
            _connectionTimer.Tick += ConnectionTimer_Tick;
            _connectionTimer.Start();

            // Run initial check
            ConnectionTimer_Tick(null, null);
        }

        private void ConnectionTimer_Tick(object sender, EventArgs e)
        {
            if (_isSampling) return; // Don't overwrite active sampling progress text

            bool connected = MainV2.comPort != null && MainV2.comPort.BaseStream != null && MainV2.comPort.BaseStream.IsOpen;
            if (connected)
            {
                if (_lblStatus.Text == "Disconnected. Connect a vehicle first." || _lblStatus.Text == "Ready for measurement.")
                {
                    _lblStatus.Text = "Ready for measurement.";
                    _lblStatus.ForeColor = Color.FromArgb(180, 184, 190);
                }
                _btnMeasure.Enabled = true;

                // Check parameter presence for each IMU in the flight controller
                bool imu1Present = MainV2.comPort.MAV.param.ContainsKey("INS_POS1_X");
                bool imu2Present = MainV2.comPort.MAV.param.ContainsKey("INS_POS2_X");
                bool imu3Present = MainV2.comPort.MAV.param.ContainsKey("INS_POS3_X");

                if (imu1Present)
                {
                    if (!_chkImu1.Enabled)
                    {
                        _chkImu1.Enabled = true;
                        _chkImu1.Checked = true;
                    }
                }
                else
                {
                    _chkImu1.Enabled = false;
                    _chkImu1.Checked = false;
                }

                if (imu2Present)
                {
                    if (!_chkImu2.Enabled)
                    {
                        _chkImu2.Enabled = true;
                        _chkImu2.Checked = true;
                    }
                }
                else
                {
                    _chkImu2.Enabled = false;
                    _chkImu2.Checked = false;
                }

                if (imu3Present)
                {
                    if (!_chkImu3.Enabled)
                    {
                        _chkImu3.Enabled = true;
                        _chkImu3.Checked = true;
                    }
                }
                else
                {
                    _chkImu3.Enabled = false;
                    _chkImu3.Checked = false;
                }
            }
            else
            {
                _lblStatus.Text = "Disconnected. Connect a vehicle first.";
                _lblStatus.ForeColor = Color.FromArgb(220, 100, 100);
                _btnMeasure.Enabled = false;

                // Disable and uncheck all target options when disconnected
                _chkImu1.Enabled = false;
                _chkImu1.Checked = false;
                _chkImu2.Enabled = false;
                _chkImu2.Checked = false;
                _chkImu3.Enabled = false;
                _chkImu3.Checked = false;
            }
        }

        private void ComPort_OnPacketReceived(object sender, MAVLink.MAVLinkMessage packet)
        {
            if (!_isSampling) return;

            // Look for RAW_IMU message
            if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.RAW_IMU)
            {
                try
                {
                    MAVLink.mavlink_raw_imu_t rawImu = (MAVLink.mavlink_raw_imu_t)packet.data;
                    lock (_lockObj)
                    {
                        _sampleSumX += rawImu.xacc;
                        _sampleSumY += rawImu.yacc;
                        _sampleSumZ += rawImu.zacc;
                        _sampleCount++;
                    }
                }
                catch { }
            }
        }

        private void SamplingTimer_Tick(object sender, EventArgs e)
        {
            if (!_isSampling) return;

            // Fallback sampling: read parsed CurrentState accelerometer data
            try
            {
                CurrentState cs = MainV2.comPort.MAV.cs;
                if (cs != null)
                {
                    _fallbackSamples.Add(new double[] { cs.ax, cs.ay, cs.az });
                }
            }
            catch { }
        }

        private void UiUpdateTimer_Tick(object sender, EventArgs e)
        {
            double elapsed = (DateTime.Now - _sampleStartTime).TotalSeconds;
            if (elapsed >= _sampleDurationSec)
            {
                FinishMeasurement();
            }
            else
            {
                int progress = (int)((elapsed / _sampleDurationSec) * 100);
                _progressBar.Value = Math.Min(100, Math.Max(0, progress));
                double remaining = _sampleDurationSec - elapsed;
                
                int count;
                lock (_lockObj) { count = _sampleCount; }
                if (count > 0)
                {
                    _lblStatus.Text = string.Format("Sampling: {0:F1}s left (Received {1} RAW_IMU packets)", remaining, count);
                }
                else
                {
                    _lblStatus.Text = string.Format("Sampling: {0:F1}s left (Using fallback telemetry, {1} samples)", remaining, _fallbackSamples.Count);
                }
            }
        }

        private void BtnMeasure_Click(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                MessageBox.Show("Autopilot is not connected! Connect a vehicle first.", "Connection Error", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            _btnMeasure.Enabled = false;
            _numX.Enabled = false;
            _numY.Enabled = false;
            _numZ.Enabled = false;

            _progressBar.Visible = true;
            _progressBar.Value = 0;
            _lblStatus.Text = "Hold the drone completely still...";

            // Request RAW_IMU message at 50Hz (20,000 microseconds)
            try
            {
                MainV2.comPort.doCommand(
                    (byte)MainV2.comPort.sysidcurrent,
                    (byte)MainV2.comPort.compidcurrent,
                    (MAVLink.MAV_CMD)511, // MAV_CMD_SET_MESSAGE_INTERVAL
                    27f,                  // MAVLINK_MSG_ID_RAW_IMU (27)
                    20000f,               // Interval in microseconds (50Hz)
                    0f, 0f, 0f, 0f, 0f,
                    false
                );
            }
            catch (Exception ex)
            {
                Console.WriteLine("CoG Estimator: Failed to request RAW_IMU rate: " + ex.Message);
            }

            lock (_lockObj)
            {
                _sampleSumX = 0;
                _sampleSumY = 0;
                _sampleSumZ = 0;
                _sampleCount = 0;
            }
            _fallbackSamples.Clear();

            _sampleStartTime = DateTime.Now;
            _isSampling = true;

            _samplingTimer.Start();
            _uiUpdateTimer.Start();
        }

        private void FinishMeasurement()
        {
            _isSampling = false;
            _samplingTimer.Stop();
            _uiUpdateTimer.Stop();

            // Restore RAW_IMU message stream interval to default/disabled
            try
            {
                MainV2.comPort.doCommand(
                    (byte)MainV2.comPort.sysidcurrent,
                    (byte)MainV2.comPort.compidcurrent,
                    (MAVLink.MAV_CMD)511, // MAV_CMD_SET_MESSAGE_INTERVAL
                    27f,                  // MAVLINK_MSG_ID_RAW_IMU (27)
                    -1f,                  // -1 = disable, 0 = reset to default stream rate
                    0f, 0f, 0f, 0f, 0f,
                    false
                );
            }
            catch { }

            _btnMeasure.Enabled = true;
            _numX.Enabled = true;
            _numY.Enabled = true;
            _numZ.Enabled = true;
            _progressBar.Visible = false;

            double finalX = 0, finalY = 0, finalZ = 0;
            bool success = false;

            int count;
            lock (_lockObj) { count = _sampleCount; }

            if (count > 20)
            {
                // We got RAW_IMU data!
                lock (_lockObj)
                {
                    finalX = _sampleSumX / count;
                    finalY = _sampleSumY / count;
                    finalZ = _sampleSumZ / count;
                }
                success = true;
                _lblStatus.Text = string.Format("Measured successfully using {0} RAW_IMU packets.", count);
            }
            else if (_fallbackSamples.Count > 10)
            {
                // Fallback to CurrentState
                double sumX = 0, sumY = 0, sumZ = 0;
                foreach (double[] sample in _fallbackSamples)
                {
                    sumX += sample[0];
                    sumY += sample[1];
                    sumZ += sample[2];
                }
                finalX = sumX / _fallbackSamples.Count;
                finalY = sumY / _fallbackSamples.Count;
                finalZ = sumZ / _fallbackSamples.Count;
                success = true;
                _lblStatus.Text = string.Format("Measured successfully using {0} fallback telemetry frames.", _fallbackSamples.Count);
            }
            else
            {
                _lblStatus.Text = "Measurement failed. No IMU data received.";
                MessageBox.Show("Failed to record telemetry data during the measurement window. Ensure the telemetry stream is active.", "Measurement Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            if (success)
            {
                // Check if the accelerometer vector is non-zero
                double mag = Math.Sqrt(finalX * finalX + finalY * finalY + finalZ * finalZ);
                if (mag < 100)
                {
                    MessageBox.Show("IMU values are near zero. Is the drone in freefall or is the sensor faulty?", "Data Validation Error", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                // Gravity vector points in direction opposite of acceleration
                double[] gravityDir = new double[3];
                gravityDir[0] = -finalX / mag;
                gravityDir[1] = -finalY / mag;
                gravityDir[2] = -finalZ / mag;

                // Create the measurement
                _measurementCounter++;
                Measurement m = new Measurement();
                m.Index = _measurementCounter;
                m.HangX = (double)_numX.Value / 1000.0; // Convert mm to meters
                m.HangY = (double)_numY.Value / 1000.0;
                m.HangZ = (double)_numZ.Value / 1000.0;
                m.AccelX = finalX;
                m.AccelY = finalY;
                m.AccelZ = finalZ;
                m.GravityDir = gravityDir;
                m.Residual = 0.0;

                _measurements.Add(m);
                UpdateMeasurementsGrid();
                CalculateCenterOfGravity();
            }
        }

        private void UpdateMeasurementsGrid()
        {
            _dataGridView.Rows.Clear();
            foreach (Measurement m in _measurements)
            {
                string hangStr = string.Format("({0:F0}, {1:F0}, {2:F0})", m.HangX * 1000.0, m.HangY * 1000.0, m.HangZ * 1000.0);
                string gravStr = string.Format("({0:F3}, {1:F3}, {2:F3})", m.GravityDir[0], m.GravityDir[1], m.GravityDir[2]);
                string resStr = m.Residual > 0.0 ? string.Format("{0:F1} mm", m.Residual * 1000.0) : "---";

                int rowIndex = _dataGridView.Rows.Add(m.Index, hangStr, gravStr, resStr);
                
                // Color code rows based on residuals if we have enough points
                if (_measurements.Count >= 3)
                {
                    if (m.Residual > 0.020) // > 20 mm
                    {
                        _dataGridView.Rows[rowIndex].DefaultCellStyle.BackColor = Color.FromArgb(70, 30, 30);
                    }
                    else if (m.Residual > 0.010) // > 10 mm
                    {
                        _dataGridView.Rows[rowIndex].DefaultCellStyle.BackColor = Color.FromArgb(70, 55, 30);
                    }
                }
            }
        }

        private void CalculateCenterOfGravity()
        {
            if (_measurements.Count < 2)
            {
                _lblCogResult.Text = "Center of Gravity (CoG) relative to IMU:\n- X (Forward): Need at least 2 measurements\n- Y (Right): ---\n- Z (Down): ---";
                _lblParamResult.Text = "Derived INS_POS offsets:\n- INS_POS_X: Need at least 2 measurements\n- INS_POS_Y: ---\n- INS_POS_Z: ---";
                _btnWriteParams.Enabled = false;
                return;
            }

            double[,] A = new double[3, 3];
            double[] B = new double[3];

            foreach (Measurement m in _measurements)
            {
                double dx = m.GravityDir[0];
                double dy = m.GravityDir[1];
                double dz = m.GravityDir[2];

                double px = m.HangX;
                double py = m.HangY;
                double pz = m.HangZ;

                // M = I - d * d^T
                double m00 = 1.0 - dx * dx;
                double m01 = -dx * dy;
                double m02 = -dx * dz;

                double m10 = -dy * dx;
                double m11 = 1.0 - dy * dy;
                double m12 = -dy * dz;

                double m20 = -dz * dx;
                double m21 = -dz * dy;
                double m22 = 1.0 - dz * dz;

                A[0, 0] += m00; A[0, 1] += m01; A[0, 2] += m02;
                A[1, 0] += m10; A[1, 1] += m11; A[1, 2] += m12;
                A[2, 0] += m20; A[2, 1] += m21; A[2, 2] += m22;

                B[0] += m00 * px + m01 * py + m02 * pz;
                B[1] += m10 * px + m11 * py + m12 * pz;
                B[2] += m20 * px + m21 * py + m22 * pz;
            }

            double[] C;
            if (Solve3x3(A, B, out C))
            {
                double cogX = C[0];
                double cogY = C[1];
                double cogZ = C[2];

                // Calculate residuals for each measurement
                for (int i = 0; i < _measurements.Count; i++)
                {
                    Measurement m = _measurements[i];
                    double vx = cogX - m.HangX;
                    double vy = cogY - m.HangY;
                    double vz = cogZ - m.HangZ;

                    double dot = vx * m.GravityDir[0] + vy * m.GravityDir[1] + vz * m.GravityDir[2];
                    double vMagSq = vx * vx + vy * vy + vz * vz;
                    double distSq = vMagSq - dot * dot;
                    
                    m.Residual = distSq > 0 ? Math.Sqrt(distSq) : 0.0;
                }

                UpdateMeasurementsGrid();

                // Display results (converting CoG to mm, parameters are kept in meters)
                _lblCogResult.Text = string.Format(
                    "Center of Gravity (CoG) relative to IMU:\n- X (Forward): {0:F1} mm\n- Y (Right): {1:F1} mm\n- Z (Down): {2:F1} mm",
                    cogX * 1000.0, cogY * 1000.0, cogZ * 1000.0
                );

                // Derived INS_POS offset (position of the IMU relative to the CoG)
                double insX = -cogX;
                double insY = -cogY;
                double insZ = -cogZ;

                _lblParamResult.Text = string.Format(
                    "Derived INS_POS offsets:\n- INS_POS_X: {0:F3} m ({1:F1} mm)\n- INS_POS_Y: {2:F3} m ({3:F1} mm)\n- INS_POS_Z: {4:F3} m ({5:F1} mm)",
                    insX, insX * 1000.0, insY, insY * 1000.0, insZ, insZ * 1000.0
                );

                _btnWriteParams.Enabled = true;
            }
            else
            {
                _lblCogResult.Text = "Center of Gravity (CoG) relative to IMU:\n- Estimation failed (ill-conditioned lines).\nHang drone at different angles.";
                _lblParamResult.Text = "Derived INS_POS offsets:\n- Estimation failed.";
                _btnWriteParams.Enabled = false;
            }
        }

        private bool Solve3x3(double[,] A, double[] B, out double[] C)
        {
            C = new double[3];
            double m00 = A[0, 0], m01 = A[0, 1], m02 = A[0, 2];
            double m10 = A[1, 0], m11 = A[1, 1], m12 = A[1, 2];
            double m20 = A[2, 0], m21 = A[2, 1], m22 = A[2, 2];

            // Determinant
            double det = m00 * (m11 * m22 - m12 * m21)
                       - m01 * (m10 * m22 - m12 * m20)
                       + m02 * (m10 * m21 - m11 * m20);

            if (Math.Abs(det) < 1e-8)
                return false;

            double invDet = 1.0 / det;

            // Adjugate matrix
            double[,] adj = new double[3, 3];
            adj[0, 0] = (m11 * m22 - m12 * m21);
            adj[0, 1] = (m02 * m21 - m01 * m22);
            adj[0, 2] = (m01 * m12 - m02 * m11);

            adj[1, 0] = (m12 * m20 - m10 * m22);
            adj[1, 1] = (m00 * m22 - m02 * m20);
            adj[1, 2] = (m02 * m10 - m00 * m12);

            adj[2, 0] = (m10 * m21 - m11 * m20);
            adj[2, 1] = (m01 * m20 - m00 * m21);
            adj[2, 2] = (m00 * m11 - m01 * m10);

            // Compute solution
            C[0] = invDet * (adj[0, 0] * B[0] + adj[0, 1] * B[1] + adj[0, 2] * B[2]);
            C[1] = invDet * (adj[1, 0] * B[0] + adj[1, 1] * B[1] + adj[1, 2] * B[2]);
            C[2] = invDet * (adj[2, 0] * B[0] + adj[2, 1] * B[1] + adj[2, 2] * B[2]);

            return true;
        }

        private void BtnDelete_Click(object sender, EventArgs e)
        {
            if (_dataGridView.SelectedRows.Count > 0)
            {
                int index = (int)_dataGridView.SelectedRows[0].Cells[0].Value;
                _measurements.RemoveAll(m => m.Index == index);
                UpdateMeasurementsGrid();
                CalculateCenterOfGravity();
            }
        }

        private void BtnClear_Click(object sender, EventArgs e)
        {
            _measurements.Clear();
            _measurementCounter = 0;
            UpdateMeasurementsGrid();
            CalculateCenterOfGravity();
        }

        private void BtnWriteParams_Click(object sender, EventArgs e)
        {
            if (_measurements.Count < 2) return;

            // Perform calculations to get offsets
            double[,] A = new double[3, 3];
            double[] B = new double[3];

            foreach (Measurement m in _measurements)
            {
                double dx = m.GravityDir[0];
                double dy = m.GravityDir[1];
                double dz = m.GravityDir[2];

                double px = m.HangX;
                double py = m.HangY;
                double pz = m.HangZ;

                double m00 = 1.0 - dx * dx;
                double m01 = -dx * dy;
                double m02 = -dx * dz;

                double m10 = -dy * dx;
                double m11 = 1.0 - dy * dy;
                double m12 = -dy * dz;

                double m20 = -dz * dx;
                double m21 = -dz * dy;
                double m22 = 1.0 - dz * dz;

                A[0, 0] += m00; A[0, 1] += m01; A[0, 2] += m02;
                A[1, 0] += m10; A[1, 1] += m11; A[1, 2] += m12;
                A[2, 0] += m20; A[2, 1] += m21; A[2, 2] += m22;

                B[0] += m00 * px + m01 * py + m02 * pz;
                B[1] += m10 * px + m11 * py + m12 * pz;
                B[2] += m20 * px + m21 * py + m22 * pz;
            }

            double[] C;
            if (!Solve3x3(A, B, out C))
            {
                MessageBox.Show("Calibration calculations failed. Cannot write parameters.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            // INS_POS is the position of the IMU relative to the CoG
            float insX = (float)(-C[0]);
            float insY = (float)(-C[1]);
            float insZ = (float)(-C[2]);

            // Construct list of parameters to write
            List<ParamVal> paramsToWrite = new List<ParamVal>();
            if (_chkImu1.Checked)
            {
                paramsToWrite.Add(new ParamVal("INS_POS1_X", insX));
                paramsToWrite.Add(new ParamVal("INS_POS1_Y", insY));
                paramsToWrite.Add(new ParamVal("INS_POS1_Z", insZ));
            }
            if (_chkImu2.Checked)
            {
                paramsToWrite.Add(new ParamVal("INS_POS2_X", insX));
                paramsToWrite.Add(new ParamVal("INS_POS2_Y", insY));
                paramsToWrite.Add(new ParamVal("INS_POS2_Z", insZ));
            }
            if (_chkImu3.Checked)
            {
                paramsToWrite.Add(new ParamVal("INS_POS3_X", insX));
                paramsToWrite.Add(new ParamVal("INS_POS3_Y", insY));
                paramsToWrite.Add(new ParamVal("INS_POS3_Z", insZ));
            }

            if (paramsToWrite.Count == 0)
            {
                MessageBox.Show("No target IMUs selected! Select at least one IMU checkbox.", "Verification", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            // Confirm with user
            string confirmMsg = string.Format(
                "Are you sure you want to write the following offsets to the flight controller?\n\n" +
                "Offsets:\n" +
                "X Offset: {0:F3} m\n" +
                "Y Offset: {1:F3} m\n" +
                "Z Offset: {2:F3} m\n\n" +
                "This will update {3} parameters on the vehicle.",
                insX, insY, insZ, paramsToWrite.Count
            );

            if (MessageBox.Show(confirmMsg, "Confirm Parameter Write", MessageBoxButtons.YesNo, MessageBoxIcon.Question) == DialogResult.Yes)
            {
                try
                {
                    int successCount = 0;
                    foreach (ParamVal param in paramsToWrite)
                    {
                        // Check if parameter exists on vehicle before writing
                        if (MainV2.comPort.MAV.param.ContainsKey(param.Name))
                        {
                            if (MainV2.comPort.setParam(param.Name, param.Value))
                            {
                                successCount++;
                            }
                        }
                        else
                        {
                            Console.WriteLine(string.Format("Autopilot doesn't support parameter {0}. Skipping.", param.Name));
                        }
                    }

                    if (successCount == paramsToWrite.Count)
                    {
                        MessageBox.Show("Successfully wrote all offset parameters to the autopilot!", "Parameter Sync", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    }
                    else if (successCount > 0)
                    {
                        MessageBox.Show(string.Format("Partially wrote parameters. Updated {0} out of {1} parameters.\nCheck flight controller connection and parameter versions.", successCount, paramsToWrite.Count), "Parameter Sync Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    }
                    else
                    {
                        MessageBox.Show("Failed to write parameters. Autopilot may not support these INS_POS parameters or the link is unstable.", "Parameter Write Failed", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error writing parameters: " + ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                // Unsubscribe from MAVLink callback and stop timers when control is disposed
                try
                {
                    MainV2.comPort.OnPacketReceived -= ComPort_OnPacketReceived;
                }
                catch { }

                try { if (_samplingTimer != null) _samplingTimer.Stop(); } catch { }
                try { if (_uiUpdateTimer != null) _uiUpdateTimer.Stop(); } catch { }
                try { if (_connectionTimer != null) _connectionTimer.Stop(); } catch { }
            }
            base.Dispose(disposing);
        }
    }
}
