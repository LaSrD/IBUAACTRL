using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CANDEVICE;
/// <summary>
/// 电机底层控制类
/// </summary>
namespace RMD_L_MOTOR
{
    public struct PAIR
    {
        public byte kp;
        public byte ki;
        public PAIR(byte kp, byte ki) : this()
        {
            this.kp = kp;
            this.ki = ki;
        }
    }
    /// <summary>
    /// 电机三环PI参数
    /// </summary>
    public struct PID
    {
        public PAIR Angle;
        public PAIR Speed;
        public PAIR Torque;   //转矩

        public PID(PAIR angle, PAIR speed, PAIR iq) : this()
        {
            this.Angle = angle;
            this.Speed = speed;
            this.Torque = iq;
        }
    }
    /// <summary>
    /// 电机型号
    /// </summary>
    public enum MOTORTYPE : short
    {
        RMD_L_5010 = 0,
        RMD_L_7010 = 1,
        RMD_L_7025 = 2
    }
    /// <summary>
    /// 电机运行状态
    /// </summary>
    public enum MOTORERRORFLAG : byte
    {
        STATE_OK = 0,
        STATE_ERROR = 1
    }
    public class RMDLMotor
    {
        /// <summary>
        /// 电机返回数据
        /// </summary>
        public PID pID = new PID(new PAIR(0, 0), new PAIR(0, 0), new PAIR(0, 0));// pid 参数
        public int Accel = 0;           //加速度   dps/s
        public double LoopyAngle = 0;   //多圈角度 度
        public double OneloopAngle = 0; //单圈角度 度
        public byte Temperature = 0;    //电机温度 摄氏度
        public int current = 0;         //转矩电流 毫安
        public short speed = 0;         //电机转速 dps
        public double voltage = 0;      //电机电压 毫伏   
        public MOTORERRORFLAG voltage_state = MOTORERRORFLAG.STATE_OK;  //电机电压状态
        public MOTORERRORFLAG temp_state = MOTORERRORFLAG.STATE_OK;     //电压温度状态
        #region 私有成员
        /// <summary>
        /// 电机ID及类型
        /// </summary>
        private uint MotorID;
        private MOTORTYPE MotorType;
        /// <summary>
        /// 电机基本参数
        /// </summary>
        private int MaxTorque;              //额定转矩 mNm
        private int NormalCurrent;          //额定电流 mA 
        private double TorqueConstant;      //转矩常数 mNm/mA
        private double TorqueConstant_min;  //转矩常数 mNm/mA
        private double TorqueBoundary;      //分段负载值：小负载时电机的堵转扭矩常数变化mNm
        /// <summary>
        /// can设备
        /// </summary>
        private CANDevice Mcandevice;
        #endregion
        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        public RMDLMotor(uint id, MOTORTYPE motor_type, CANDevice cANDevice) 
        {
            this.MotorID = id;
            this.MotorType = motor_type;
            this.Mcandevice = cANDevice;

            if (this.MotorType == MOTORTYPE.RMD_L_5010)
            {
                MaxTorque = 260;
                NormalCurrent = 1650;
                TorqueConstant = 0.16;
                TorqueConstant_min = 0.16;
                TorqueBoundary = 70;
            }
            else if (this.MotorType == MOTORTYPE.RMD_L_7010)
            {
                MaxTorque = 630;
                NormalCurrent = 1750;
                TorqueConstant = 0.16;
                TorqueConstant_min = 0.16;
                TorqueBoundary = 70;
            }
            else if (this.MotorType == MOTORTYPE.RMD_L_7025)
            {
                MaxTorque = 1600;
                NormalCurrent = 3000;
                TorqueConstant = 1.0;
                TorqueConstant_min = 1.0;
                TorqueBoundary = 170;
            }
        }
        /// <summary>
        /// 电机上电连接状态
        /// </summary>
        /// <returns></returns>
        public bool isConnect()
        {
            if (Mcandevice.GetConnectState() == true)
            {
                byte[] data = { 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                Mcandevice.CAN_Tranmit(this.MotorID, data);
                if (Mcandevice.ReceiveDate.data != null 
                    && Mcandevice.ReceiveDate.ID == this.MotorID 
                    && Mcandevice.ReceiveDate.data[0] == 0x30)
                {
                    return true;
                }
            }
            return false;
        }
        /// <summary>
        /// 读取pid 参数
        /// </summary>
        public void ReadPID()
        {
            if (Mcandevice.GetConnectState() == false) return;

            byte[] data = { 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x30)
            {
                byte[] tempdata = Mcandevice.ReceiveDate.data;

                this.pID.Angle.kp = tempdata[2];
                this.pID.Angle.ki = tempdata[3];
                this.pID.Speed.kp = tempdata[4];
                this.pID.Speed.ki = tempdata[5];
                this.pID.Torque.kp = tempdata[6];
                this.pID.Torque.ki = tempdata[7];
            }
        }
        /// <summary>
        /// 写入pid参数 
        /// </summary>
        public void WritePID(PID _pID)
        {
            if (Mcandevice.GetConnectState() == false) return;
            byte[] data = { 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

            data[2] = _pID.Angle.kp;
            data[3] = _pID.Angle.ki;
            data[4] = _pID.Speed.kp;
            data[5] = _pID.Speed.ki;
            data[6] = _pID.Torque.kp;
            data[7] = _pID.Torque.ki;

            Mcandevice.CAN_Tranmit(this.MotorID, data);
        }
        public void WritePID()
        {
            if (Mcandevice.GetConnectState() == false) return;
            byte[] data = { 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

            data[2] = 100;
            data[3] = 100;
            data[4] = 100;
            data[5] = 100;
            data[6] = 100;
            data[7] = 100;
            Mcandevice.CAN_Tranmit(this.MotorID, data);
        }
        /// <summary>
        /// 读取电机加速度
        /// </summary>
        public void ReadAccel()
        {
            if (Mcandevice.GetConnectState() == false) return;
            byte[] data = { 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x33)
            {
                byte[] tempdata = Mcandevice.ReceiveDate.data;

                for (int i = 7; i >= 4; --i)
                {
                    this.Accel <<= 8;
                    this.Accel |= tempdata[i];
                }
            }
        }
        /// <summary>
        /// 写入电机加速度
        /// </summary>
        /// <param name="accel">加速度 int</param>
        public void WriteAccel(int accel)
        {

        }
        /// <summary>
        /// 读取编码器数据
        /// </summary>
        public void ReadEncode()
        {

        }
        /// <summary>
        /// 写入编码器值作为电机零点
        /// </summary>
        /// <param name="encodedata">编码器值</param>
        /// <returns>成功标志</returns>
        public bool WriteEncodeToZero(short encodedata)
        {
            return false;
        }
        /// <summary>
        /// 写入当前位置作为零点 , 电机的驱动并未实现
        /// </summary>
        /// <returns></returns>
        public bool SetCurToZero()
        {
            if (Mcandevice.GetConnectState() == false) return false;

            byte[] data = new byte[8] { 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x90)
            {
                byte[] tempdata = Mcandevice.ReceiveDate.data;
                byte[] senddata = { 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                senddata[6] = tempdata[4];
                senddata[7] = tempdata[5];
                Mcandevice.CAN_Tranmit(this.MotorID, senddata);
                return true;
            }
            return false;

        }
        /// <summary>
        /// 读取多圈角度
        /// </summary>
        public void ReadLoopyAngle()
        {
            Int32 tempAngle = 0;
            if (Mcandevice.GetConnectState() == false) return;
          //  if (this.State == false) return;
            byte[] data = new byte[8] { 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x92)
            {
                byte[] tempdata = Mcandevice.ReceiveDate.data;

                for (int i = 7; i >= 1; --i)
                {
                    tempAngle <<= 8;
                    tempAngle |= tempdata[i];
                }
                LoopyAngle = tempAngle / 100.0f;
            }
        }
        /// <summary>
        /// 单圈角度
        /// </summary>
        public void ReadOneLoopyAngle()
        {
            ushort tempAngle = 0;
            if (Mcandevice.GetConnectState() == false) return;
           // if (this.State == false) return;
            byte[] data = new byte[8] { 0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x94)
            {

                byte[] tempdata = new byte[8];
                tempdata = Mcandevice.ReceiveDate.data;

                if (tempdata != null)
                {
                    for (int i = 7; i >= 6; --i)
                    {
                        tempAngle <<= 8;
                        tempAngle |= tempdata[i];
                    }
                    OneloopAngle = tempAngle / 100.0f;
                }
            }
        }
        /// <summary>
        /// 读取电机的错误标志
        /// </summary>
        public void ReadMotorError()
        {
            if (Mcandevice.GetConnectState() == false) return;
           // if (this.State == false) return;
            byte temp = 0;
            byte[] data = new byte[8] { 0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x9A)
            {
                byte[] tempdata = new byte[8] { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                tempdata = Mcandevice.ReceiveDate.data;

                if (tempdata != null)
                {
                    temp = tempdata[7];
                }
                this.voltage_state = (MOTORERRORFLAG)(temp & 0x01);
                this.temp_state = (MOTORERRORFLAG)(temp & 0x40);
            }
        }
        /// <summary>
        /// 读取电机的温度、电流、转速、电压
        /// </summary>
        public void ReadMotorSate()
        {
            if (Mcandevice.GetConnectState() == false) return;
          //  if (this.State == false) return;
            byte[] data = new byte[8] { 0x9c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);


            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x9c)
            {
                byte[] tempdata = new byte[8] { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                tempdata = Mcandevice.ReceiveDate.data;

                if (tempdata != null)
                {
                    //温度
                    this.Temperature = tempdata[1];
                    //电流
                    short tempcurrent = 0;
                    for (int i = 3; i >= 2; --i)
                    {
                        tempcurrent <<= 8;
                        tempcurrent |= (short)tempdata[i];
                    }
                    this.current = (int)(((double)tempcurrent) * (33000.0 / 2048));
                    //速度
                    short tempSpeed = 0;
                    for (int i = 5; i >= 4; --i)
                    {
                        tempSpeed <<= 8;
                        tempSpeed |= (short)(tempdata[i]);
                    }
                    this.speed = (short)tempSpeed;
                }
            }
            byte[] data2 = new byte[8] { 0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data2);
            if (Mcandevice.ReceiveDate.data != null && Mcandevice.ReceiveDate.ID == this.MotorID && Mcandevice.ReceiveDate.data[0] == 0x9A)
            {
                byte[] tempdata = new byte[8] { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                tempdata = Mcandevice.ReceiveDate.data;
                short tempvoltage = 0;
                for (int i = 4; i >= 3; --i)
                {
                    tempvoltage <<= 8;
                    tempvoltage |= (short)tempdata[i];
                }
                this.voltage = tempvoltage / 10.0f;
            }
        }

        /// <summary>
        /// 关闭电机
        /// </summary>
        /// <returns></returns>
        public bool CloseMotor()
        {
            if (Mcandevice.GetConnectState() == false) return false;

            byte[] data = new byte[8] { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            return true;
        }
        /// <summary>
        /// 停止电机
        /// </summary>
        /// <returns></returns>
        public bool StopMotor()
        {
            if (Mcandevice.GetConnectState() == false) return false;

            byte[] data = new byte[8] { 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
           // this.State = false;
            byte[] data2 = new byte[8] { 0xa1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data2);
            Mcandevice.CAN_Tranmit(this.MotorID, data);

            return true;
        }
        /// <summary>
        /// 运行电机，从停止状态恢复停止前的控制方式
        /// </summary>
        /// <returns></returns>
        public bool StartMotor()
        {
            if (Mcandevice.GetConnectState() == false) return false;

            byte[] data = new byte[8] { 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            Mcandevice.CAN_Tranmit(this.MotorID, data);
           // this.State = true;

            if (Mcandevice.ReceiveDate.data == null || Mcandevice.ReceiveDate.data[0] != 0x88) return false;

            return true;
        }
        /// <summary>
        /// 转矩闭环控制
        /// </summary>
        /// <param name="iq">转矩大小 单位mNm</param>
        public void iqCotrol(double iq)
        {
            if (Mcandevice.GetConnectState() == false) return;
         //   if (this.State == false) return;

            double torqueConstant = 0;
            if (iq < this.TorqueBoundary)
                torqueConstant = this.TorqueConstant_min;
            else
                torqueConstant = this.TorqueConstant;

            double current = iq / torqueConstant;  //转矩对应的电流大小 mA
            short temp = (short)(2000 * (current / 32000));

            byte[] data = new byte[8] { 0xA1, 0, 0, 0, 0, 0, 0, 0 };
            data[4] = (byte)(temp & 0x00ff);
            data[5] = (byte)(temp >> 8 & 0x00ff);

            Mcandevice.CAN_Tranmit(this.MotorID, data);

        }
        /// <summary>
        /// 速度闭环控制
        /// </summary>
        /// <param name="speed">转速 dps</param>
        public void speedCotrol(int speed)
        {

        }
        /// <summary>
        /// 位置环控制，单圈绝对位置控制
        /// </summary>
        /// <param name="AngleControl_">目标位置 ，度</param>
        /// <param name="spinDirection">从当前位置到达目标位置的旋转方向： 00: 顺时针（正）、01: 逆时针</param>
        /// <param name="maxspeed">限制电机的最大转速dps</param>
        public void AngleControl_A6(double AngleControl_, short maxspeed, byte spinDirection)
        {
            if (Mcandevice.GetConnectState() == false) return;
      //      if (this.State == false) return;

            short Angle = (short)(AngleControl_ * 100);

            byte[] data = new byte[8] { 0xA6, 0, 0, 0, 0, 0, 0, 0 };
            if (spinDirection > 1)
                return;
            data[1] = spinDirection;
            data[2] = (byte)(maxspeed & 0x00ff);
            data[3] = (byte)((maxspeed >> 8) & 0x00ff);
            data[4] = (byte)(Angle & (0x00ff));
            data[5] = (byte)((Angle >> 8) & (0x00ff));

            Mcandevice.CAN_Tranmit(this.MotorID, data);
        }
        /// <summary>
        /// 位置闭环控制，多圈绝对位置控制
        /// </summary>
        /// <param name="AngleControl_">目标位置</param>
        /// <param name="maxspeed">限制电机的最大转速dps</param>
        public void AngleControl_A4(double AngleControl_, short maxspeed)
        {
            if (Mcandevice.GetConnectState() == false) return;
            //      if (this.State == false) return;

            int Angle = (int)(AngleControl_ * 100);

            byte[] data = new byte[8] { 0xA4, 0, 0, 0, 0, 0, 0, 0 };
            data[2] = (byte)(maxspeed & 0x00ff);
            data[3] = (byte)((maxspeed >> 8) & 0x00ff);
            data[4] = (byte)(Angle & (0x00ff));
            data[5] = (byte)((Angle >> 8) & (0x00ff));
            data[6] = (byte)((Angle >> 16) & (0x00ff));
            data[7] = (byte)((Angle >> 24) & (0x00ff));

            Mcandevice.CAN_Tranmit(this.MotorID, data);
        }
        /// <summary>
        /// 位置控制：增量位置
        /// </summary>
        /// <param name="AngleControl_">位置增量，正：顺时针，负：逆时针</param>
        /// <param name="maxspeed">限制电机的最大转速 dps</param>
        public void increAngleControl(double AngleControl_, short maxspeed)
        {
            if (Mcandevice.GetConnectState() == false) return;
     //       if (this.State == false) return;

            int Angle = (int)(AngleControl_ * 100);

            byte[] data = new byte[8] { 0xA8, 0, 0, 0, 0, 0, 0, 0 };
            data[2] = (byte)(maxspeed & 0x00ff);
            data[3] = (byte)((maxspeed >> 8) & 0x00ff);
            data[4] = (byte)(Angle & (0x00ff));
            data[5] = (byte)((Angle >> 8) & (0x00ff));
            data[6] = (byte)((Angle >> 16) & (0x00ff));
            data[7] = (byte)((Angle >> 24) & (0x00ff));
            Mcandevice.CAN_Tranmit(this.MotorID, data);
        }
    }
    #endregion

}
