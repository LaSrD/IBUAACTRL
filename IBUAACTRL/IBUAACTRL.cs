using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RMD_L_MOTOR;
using CANDEVICE;
using System.Threading;


namespace IBUAACTRL
{
    /// <summary>
    /// 设备打开状态
    /// </summary>
    public enum DeviceOpenStatus : short
    {
        CloseFault = 0,
        CloseSuccess = 1,
        OpenFault = 2,
        InitFault = 3,
        StartFault = 4,
        StartSuccess = 5,
        ERRO = 6,
        ResetSuccess = 7,
        ConnectFault = 8
    }
    public struct DEVICEINFO
    {
        public string hw_type;
        public string Serial_Num;
        public string Procotol_type;
        public string Ctrl_Num;
        public DEVICEINFO(string procotol,string ctrlnum)
        {
            hw_type = "";
            Serial_Num = "";
            Procotol_type = procotol;
            Ctrl_Num = ctrlnum;
        }
    }
    public class IBuaaDevice
    {
        /// <summary>
        /// 手柄串口
        /// </summary>
       // public PortControl PortHande;
        public DEVICEINFO DeviceInfo = new DEVICEINFO("CAN_Baud:1000K","iBuaaControl_v0.1");

        /// <summary>
        /// 机械臂关节转角
        /// </summary>
        public double RobotAngle_Base;
        public double RobotAngle_Uparm;
        public double RobotAngle_LowArm;
        public double RobotAngle_Wirst;
        public double RobotAngle_WirstR;

        /// <summary>
        /// 机械臂末端位姿
        /// </summary>
        /// <a name = "end"></a>>
        public double px;
        public double py;
        public double pz;
        public double roll;     //回转 横滚
        public double pitch;    // 俯仰
        public double yaw;      // 偏转 偏航
        /// <summary>
        /// 电机电流
        /// </summary>
        public int[] curren = { 0, 0, 0, 0, 0 };  // 各电机电流
        public byte[] temper = { 0, 0, 0, 0, 0 }; //各电机温度
        #region 私有成员
        private CANDevice cANDevice = new CANDevice();
        /// <summary>
        /// 电机映射
        /// </summary>
        private RMDLMotor Motor_Base;    //底座电机
        private RMDLMotor Motor_UpArm;   //大臂电机
        private RMDLMotor Motor_LowArm;  //小臂电机
        private RMDLMotor Motor_Wirst;   //腕电机
        private RMDLMotor Motor_WirstR;  //腕自转臂电机
        /// <summary>
        /// 电机输出转矩 ,私有成员调试结束改回去
        /// </summary>
        public double Toruqe_Base = 0;
        public double Toruqe_UpArm = 0;
        public double Toruqe_LowArm = 0;
        public double Toruqe_Wirst = 0;
        public double Toruqe_WirstR = 0;
        /// <summary>
        /// 机械臂关节角限制
        /// </summary>
        private double UPARMANGLEMAX = 2;
        private double UPARMANGLEMIN = -90;
        private double LOWARANGLEMAX =  100;
        private double LOWARANGLEMIN = -100;
        private double WIRSTANGLEMAX = 100;
        private double WIRSTANGLEMIN = -100;
        /// <summary>
        /// 质量
        /// </summary>
        private double GLoDo = 9.80655;       //重力加速度
        private double Mass_Wirst = 0.3894;   //kg  20220317 修改（构型变化）
        private double Mass_LowArm = 0.3947;
        private double Mass_UpArm = 0.8948;
        /// <summary>
        /// 质点位置
        /// </summary>
        private double Particleh5 = 122.5; // 末端据 4，5坐标系的距离 mm
        private double Particleh4 = 115.4; //wirst质点位置 单位： mm
        private double Particleh3 = 14.57;
        private double Particleh2 = 72.55;
        /// <summary>
        /// 摩擦矩补偿 mNm
        /// </summary>
        //  private double FrictionTorque_wirst = 50; 
        /// <summary>
        /// 连杆长度
        /// </summary>
        //private double LowArmLinkLenght = 0.1122;   20220317 注释
        //private double UpArmLinkLenght = 0.1533;
        private double d3 = 217.5;         // 关节2与关节4之间的距离 mm
        /// <summary>
        /// 电流环true、位置环false按键切换标识
        /// </summary>
        private bool SwitchFlag = true;
        /// <summary>
        /// 线程
        /// </summary>
        private Thread canReceiveThread;
        private bool InitFlag = false;
        private bool ConnectFlag = false;

        private float m_ratio;  // 大臂减速比

        #endregion
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="ratio">关节2 的减速比 ： 默认为 3</param>
        public IBuaaDevice(float ratio)
        {
            this.m_ratio = ratio;
        }
        #region 方法接口
        /// <summary>
        /// 初始化
        /// </summary>
        /// <returns></returns>
        public void Init(uint[] id)
        {
            Motor_Base = new RMDLMotor(id[4], MOTORTYPE.RMD_L_7025, this.cANDevice);
            Motor_UpArm = new RMDLMotor(id[3], MOTORTYPE.RMD_L_7025, this.cANDevice);
            Motor_LowArm = new RMDLMotor(id[2], MOTORTYPE.RMD_L_7025, this.cANDevice);
            Motor_Wirst = new RMDLMotor(id[1], MOTORTYPE.RMD_L_7010, this.cANDevice);
            Motor_WirstR = new RMDLMotor(id[0], MOTORTYPE.RMD_L_5010, this.cANDevice);

            InitFlag = true;
            canReceiveThread = new Thread(new ThreadStart(canReceive));
            canReceiveThread.IsBackground = true;
            canReceiveThread.Start();
        }
        /// <summary>
        /// 连接设备
        /// </summary>
        /// <returns></returns>
        public DeviceOpenStatus Connect()
        {
            DeviceOpenStatus dStatus = (DeviceOpenStatus)cANDevice.CANCONNECT();
            if (dStatus == DeviceOpenStatus.StartSuccess)
            {
                ConnectFlag = true;
                this.cANDevice.GetBoardInfo();
                this.DeviceInfo.hw_type = cANDevice.cANDEVICEINFO.hw_type;
                this.DeviceInfo.Serial_Num = cANDevice.cANDEVICEINFO.Serial_Num;
            }
            return dStatus;
        }
        /// <summary>
        /// 获取机械臂的各关节角，关节角映射
        /// </summary>, 
        /// <param name="reducratio"></param>
        public void GetRobotAngle()
        {
            if (InitFlag == false || ConnectFlag == false)
                return;

            RobotAngle_Base = Motor_Base.LoopyAngle;
            RobotAngle_Uparm = -(Motor_UpArm.LoopyAngle / m_ratio);
            RobotAngle_LowArm = Motor_LowArm.LoopyAngle;
            RobotAngle_Wirst = -Motor_Wirst.LoopyAngle;
            RobotAngle_WirstR = Motor_WirstR.LoopyAngle;
        }
        /// <summary>
        /// 获取机械臂各个电机的电流|温度状态
        /// </summary>
        public void GetState()
        {
            if (ConnectFlag == false)
                return;
            Motor_WirstR.ReadMotorSate();
            Motor_Wirst.ReadMotorSate();
            Motor_LowArm.ReadMotorSate();
            Motor_UpArm.ReadMotorSate();
            Motor_Base.ReadMotorSate();

            this.curren[0] = Motor_WirstR.current;
            this.curren[1] = Motor_Wirst.current;
            this.curren[2] = Motor_LowArm.current;
            this.curren[3] = Motor_UpArm.current;
            this.curren[4] = Motor_Base.current;

            this.temper[0] = Motor_WirstR.Temperature;
            this.temper[1] = Motor_Wirst.Temperature;
            this.temper[2] = Motor_LowArm.Temperature;
            this.temper[3] = Motor_UpArm.Temperature;
            this.temper[4] = Motor_Base.Temperature;
        }

        /// <summary>
        /// 获取机械臂末端的位姿，结果在px,py,pz，单位mm
        /// </summary>
        public void GetEndpose()
        {
            double theta1 = System.Math.PI * RobotAngle_Base / 180.0f;
            double theta2 = System.Math.PI * RobotAngle_Uparm / 180.0f;
            double theta3 = System.Math.PI * RobotAngle_LowArm / 180.0f;
            double theta4 = System.Math.PI * RobotAngle_Wirst / 180.0f;
            double theta5 = System.Math.PI * RobotAngle_WirstR / 180.0f;

            double c1 = Math.Cos(theta1), c2 = Math.Cos(theta2), c3 = Math.Cos(theta3), c4 = Math.Cos(theta4), c5 = Math.Cos(theta5);
            double s1 = Math.Sin(theta1), s2 = Math.Sin(theta2), s3 = Math.Sin(theta3), s4 = Math.Sin(theta4), s5 = Math.Sin(theta5);

            //位置
            this.px = c1 * d3 * s2 - Particleh5 * (s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2);
            this.py = Particleh5 * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s1 * s2) + d3 * s1 * s2;
            this.pz = - c2 * d3 - Particleh5 * (c2 * c4 - c3 * s2 * s4) + 340.0f;

            //姿态
            double nx = -c5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) - s5 * (c3 * s1 + c1 * c2 * s3);
            double ny = c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) + s5 * (c1 * c3 - c2 * s1 * s3);
            double nz = c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5;
            double ox = s5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) - c5 * (c3 * s1 + c1 * c2 * s3);
            double oy = c5 * (c1 * c3 - c2 * s1 * s3) - s5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4);
            double oz = -s5 * (c2 * s4 + c3 * c4 * s2) - c5 * s2 * s3;
            double ax = s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2;
            double ay = -s4 * (c1 * s3 + c2 * c3 * s1) - c4 * s1 * s2;
            double az = c2 * c4 - c3 * s2 * s4;

            double cosbetha = System.Math.Sqrt(nx * nx + ny * ny);

            if(cosbetha != 0)
            {
                this.roll = System.Math.Atan2(ny, nx) * 180.0f / System.Math.PI;
                this.pitch = System.Math.Atan2(-nz, cosbetha) * 180.0f / System.Math.PI;
                this.yaw = System.Math.Atan2(oz, az) * 180.0f / System.Math.PI;
            }
            else if(nx * ny < 0)
            {
                this.pitch = 90.0f;
                this.roll = 0.0f;
                this.yaw = System.Math.Atan2(oz, az) * 180.0f / System.Math.PI;
            }
            else
            {
                this.pitch = -90.0f;
                this.roll = 0.0f;
                this.yaw = -System.Math.Atan2(oz, az) * 180.0f / System.Math.PI;
            }

        

        }
        /// <summary>
        /// 电机pid切换，不建议使用
        /// </summary>
        /// <param name="flag">1： 电流环PID, 0:位置环PID</param>
        private void MotorPidSwitch(bool flag)
        {

            if (flag)
            {

                Motor_Base.WritePID(new PID(new PAIR(5, 1), new PAIR(40, 30), new PAIR(10, 1)));
                Motor_UpArm.WritePID(new PID(new PAIR(5, 1), new PAIR(92, 30), new PAIR(55, 1)));
                Motor_LowArm.WritePID(new PID(new PAIR(3, 2), new PAIR(60, 30), new PAIR(10, 1)));
                Motor_Wirst.WritePID(new PID(new PAIR(5, 1), new PAIR(40, 30), new PAIR(50, 50)));
                Motor_WirstR.WritePID(new PID(new PAIR(50, 50), new PAIR(40, 30), new PAIR(50, 50)));
            }
            else
            {
                Motor_Base.WritePID(new PID(new PAIR(5, 1), new PAIR(100, 30), new PAIR(170, 1)));
                Motor_UpArm.WritePID(new PID(new PAIR(5, 1), new PAIR(92, 40), new PAIR(55, 2)));
                Motor_LowArm.WritePID(new PID(new PAIR(3, 2), new PAIR(60, 30), new PAIR(150, 1)));
                Motor_Wirst.WritePID(new PID(new PAIR(5, 1), new PAIR(100, 50), new PAIR(100, 10)));
                Motor_WirstR.WritePID(new PID(new PAIR(50, 50), new PAIR(40, 30), new PAIR(50, 50)));
            }
        }

        /// <summary>
        /// 机械臂力渲染模式
        /// </summary>
        /// <param name="fx_wcs">世界坐标下x方向的力，单位 n,默认为0</param>
        /// <param name="fy_wcs">世界坐标下y方向的力，单位 n，默认为0</param>
        /// <param name="fz_wcs">世界坐标下z方向的力，单位 n，默认为0</param>
        /// <param name="mx"></param>
        /// <param name="my"></param>
        /// <param name="mz"></param>
        public void GravityFrictionCompensation(double fx_wcs = 0, double fy_wcs = 0, double fz_wcs = 0, double mx = 0, double my = 0, double mz = 0)
        {
            double m4g = GLoDo * Mass_Wirst;
            double m3g = GLoDo * Mass_LowArm;
            double m2g = GLoDo * Mass_UpArm;
            double m2h2g = m2g * Particleh2;
            double m3h3g = m3g * Particleh3;
            double m4h4g = m4g * Particleh4;

            //double m4g = 0;
            //double m3g = 0;
            //double m2g = 0;
            //double m2h2g = 0;
            //double m3h3g = 0;
            //double m4h4g = 0;

            double theta1 = System.Math.PI * RobotAngle_Base / 180.0f;
            double theta2 = System.Math.PI * RobotAngle_Uparm / 180.0f;
            double theta3 = System.Math.PI * RobotAngle_LowArm / 180.0f;
            double theta4 = System.Math.PI * RobotAngle_Wirst / 180.0f;
            double theta5 = System.Math.PI * RobotAngle_WirstR / 180.0f;
            double c1 = Math.Cos(theta1), c2 = Math.Cos(theta2), c3 = Math.Cos(theta3), c4 = Math.Cos(theta4), c5 = Math.Cos(theta5);
            double s1 = Math.Sin(theta1), s2 = Math.Sin(theta2), s3 = Math.Sin(theta3), s4 = Math.Sin(theta4), s5 = Math.Sin(theta5);

            //有末端的力 求解世界坐标系的力
            double fx = fz_wcs * (c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5) - fx_wcs * (c5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) + s5 * (c3 * s1 + c1 * c2 * s3)) + fy_wcs * (c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) + s5 * (c1 * c3 - c2 * s1 * s3));
            double fy = fx_wcs * (s5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) - c5 * (c3 * s1 + c1 * c2 * s3)) - fz_wcs * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) - fy_wcs * (s5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) - c5 * (c1 * c3 - c2 * s1 * s3));
            double fz = fx_wcs * (s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2) - fy_wcs * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s1 * s2) + fz_wcs * (c2 * c4 - c3 * s2 * s4);

            Toruqe_WirstR = 0;
            Toruqe_Wirst = -(m4h4g * (c2 * s4 + c3 * c4 * s2) - s5 * (mx - fz * Particleh5) - c5 * my);
            Toruqe_LowArm = - c4 * (mz + fx * Particleh5) - s4 * (my * s5 - c5 * (mx - fz * Particleh5) + m4h4g * s2 * s3);
            Toruqe_UpArm = -(s3 * (c4 * (my * s5 - c5 * (mx - fz * Particleh5) + m4h4g * s2 * s3) + s4 * (mz + fx * Particleh5) - m3h3g * s2 * s3) - c3 * (c5 * my + s5 * (mx - fz * Particleh5) - m4h4g * (c2 * s4 + c3 * c4 * s2) + c3 * m3h3g * s2) + d3 * (c3 * (c4 * (c5 * fx - fy * s5 + m4g * (c2 * s4 + c3 * c4 * s2)) - s4 * (fz + m4g * (c2 * c4 - c3 * s2 * s4)) + c3 * m3g * s2) - s3 * (c5 * fy + fx * s5 - m3g * s2 * s3 - m4g * s2 * s3)) + m2h2g * s2);

            Toruqe_Base = -c2 * (s4 * (my * s5 - c5 * (mx - fz * Particleh5) + m4h4g * s2 * s3) - c4 * (mz + fx * Particleh5)) - s2 * (c3 * (c4 * (my * s5 - c5 * (mx - fz * Particleh5) + m4h4g * s2 * s3) + s4 * (mz + fx * Particleh5) - m3h3g * s2 * s3) + s3 * (c5 * my + s5 * (mx - fz * Particleh5) - m4h4g * (c2 * s4 + c3 * c4 * s2) + c3 * m3h3g * s2) - d3 * (s3 * (c4 * (c5 * fx - fy * s5 + m4g * (c2 * s4 + c3 * c4 * s2)) - s4 * (fz + m4g * (c2 * c4 - c3 * s2 * s4)) + c3 * m3g * s2) + c3 * (c5 * fy + fx * s5 - m3g * s2 * s3 - m4g * s2 * s3)));

            //if (Motor_UpArm.speed < - 3)
            //{
            //    Toruqe_UpArm = 0;
            //} 
            //if(Toruqe_LowArm * Motor_LowArm.speed < 0 && System.Math.Abs(Motor_LowArm.speed) > 5)
            //{
            //    Toruqe_LowArm = 0;
            //}

            Motor_Base.iqCotrol(Toruqe_Base);
            Motor_UpArm.iqCotrol(Toruqe_UpArm * 0.33);
            Motor_LowArm.iqCotrol(Toruqe_LowArm * 1.7);
            Motor_Wirst.iqCotrol(Toruqe_Wirst * 1.3);
            Motor_WirstR.iqCotrol(Toruqe_WirstR);

        }
        /// <summary>
        /// 机械臂单关节控制
        /// </summary>
        /// <param name="angle">0~1: 底—>腕</param>
        public void AbsoAngleControl(double[] angle)
        {
            if (angle == null || angle.Length < 5)
                return;
            double motorangle_base;
            double motorangle_up;
            double motorangle_low;
            double motorangle_wirst;
            double motorangle_wirstR;

            //电机角度映射，范围限制
            motorangle_base = angle[4];

            angle[3] = angle[3] < UPARMANGLEMIN ? UPARMANGLEMIN : angle[3];
            angle[3] = angle[3] > UPARMANGLEMAX ? UPARMANGLEMAX : angle[3];
            motorangle_up = (0 - angle[3]) * m_ratio;

            angle[2] = angle[2] > LOWARANGLEMAX ? LOWARANGLEMAX : angle[2];
            angle[2] = angle[2] < LOWARANGLEMIN ? LOWARANGLEMIN : angle[2];
            motorangle_low = angle[2];

            angle[1] = angle[1] > WIRSTANGLEMAX ? WIRSTANGLEMAX : angle[1];
            angle[1] = angle[1] < WIRSTANGLEMIN ? WIRSTANGLEMIN : angle[1];
            motorangle_wirst = 0 - angle[1];

            motorangle_wirstR = angle[0];

            Motor_Base.AngleControl_A4(motorangle_base, 1000);
            Motor_UpArm.AngleControl_A4(motorangle_up, 1000);
            Motor_LowArm.AngleControl_A4(motorangle_low, 1000);
            Motor_Wirst.AngleControl_A4(motorangle_wirst, 5000);
            Motor_WirstR.AngleControl_A4(motorangle_wirstR, 6000);
        }

        /// <summary>
        /// 切换为位置环（绝对位置），锁定机械臂姿态
        /// </summary>
        public void PostureLock_A4()
        {
            Motor_Base.AngleControl_A4(Motor_Base.LoopyAngle, 10);
            Motor_UpArm.AngleControl_A4(Motor_UpArm.LoopyAngle, 10);
            Motor_LowArm.AngleControl_A4(Motor_LowArm.LoopyAngle, 10);
            Motor_Wirst.AngleControl_A4(Motor_Wirst.LoopyAngle, 10);
            Motor_WirstR.AngleControl_A4(Motor_WirstR.LoopyAngle, 10);
        }
        /// <summary>
        /// 切换为位置环（增量0），锁定机械臂姿态
        /// </summary>
        public void PostureLock_A8()
        {
            Motor_Base.increAngleControl(0, 10);
            Motor_UpArm .increAngleControl(0, 10);
            Motor_LowArm.increAngleControl(0, 10);
            Motor_Wirst.increAngleControl(0, 10);
            Motor_WirstR.increAngleControl(0, 10);
        }
        public void WirstAngleControl(double angle)
        {
            Motor_Wirst.increAngleControl(angle, 10);
        }
        public void WirstRAngleControl(double angle)
        {
            Motor_WirstR.increAngleControl(angle, 10);
        }
        /// <summary>
        /// 增量位置控制
        /// </summary>
        /// <param name="increangle"></param>
        public void IncreAngleControl(double[] increangle)
        {
            if (increangle == null || increangle.Length < 5)
                return;
            //Motor_Base.increAngleControl(RobotAngle_Base, 10);
           // Motor_UpArm.increAngleControl(RobotAngle_Up, 10);
           // Motor_LowArm.increAngleControl(RobotAngle_Low, 10);
            //Motor_Wirst.increAngleControl(RobotAngle_Wirst, 10);
            //Motor_WirstR.increAngleControl(increangle[0], 1000);
        }
        /// <summary>
        /// 主端控制方法，接收从端的角度数据，输出控制从端需要运动的角度增量
        /// </summary>
        public double[] RobotAnglesToArray()
        {
            double[] angle = { this.RobotAngle_WirstR, this.RobotAngle_Wirst, this.RobotAngle_LowArm, this.RobotAngle_Uparm, this.RobotAngle_Base };
            return angle;
        }
        /// <summary>
        /// 判断机械臂姿态是否与给定的角度相同
        /// </summary>
        /// <param name="slaveangle"></param>
        /// <returns></returns>
        private bool isSame(double[] slaveangle)
        {
            if (slaveangle != null && slaveangle.Length == 5 
                && Math.Abs(this.RobotAngle_WirstR - slaveangle[0]) < 0.5
                && Math.Abs(this.RobotAngle_Wirst - slaveangle[1]) < 0.5
                && Math.Abs(this.RobotAngle_LowArm - slaveangle[2]) < 0.5
                && Math.Abs(this.RobotAngle_Uparm - slaveangle[3]) < 0.5
                && Math.Abs(this.RobotAngle_Base - slaveangle[4]) < 0.5
                )
                return true;
            return false;
        }
        /// <summary>
        /// 关闭设备
        /// </summary>
        public void CloseDevice()
        {
            ConnectFlag = false;
            cANDevice.CANCLOSE();
        }
        #endregion
        #region 私有成员函数
        private void canReceive()
        {
            while(true)
            {
                if (ConnectFlag == false)
                    continue;
                this.cANDevice.CANREVICE();

                Motor_WirstR.ReadLoopyAngle();
                Motor_Wirst.ReadLoopyAngle();
                Motor_LowArm.ReadLoopyAngle();
                Motor_UpArm.ReadLoopyAngle();
                Motor_Base.ReadLoopyAngle();
            }
        }
        #endregion
    }
}
