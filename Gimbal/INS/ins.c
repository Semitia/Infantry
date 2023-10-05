/**
 ******************************************************************************
 * @file    ins.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 * copied from https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example
 ******************************************************************************
 */
#include "ins.h"

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint8_t ins_debug_mode = 0;
float RefTemp = 40;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void initINS(INS_t *ins)
{
    // IMU_Param.scale[X] = 1;
    // IMU_Param.scale[Y] = 1;
    // IMU_Param.scale[Z] = 1;
    // IMU_Param.Yaw = 0;
    // IMU_Param.Pitch = 0;
    // IMU_Param.Roll = 0;
    // IMU_Param.flag = 1;
	imuInit(&ins->imu);
    ins->mutex = xSemaphoreCreateMutex();
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);

    ins->AccelLPF = 0.0085;
    ins->PitchSpeedLPF = 1.0;
    ins->YawSpeedLPF = 1.0;
}

void updateINS(INS_t *ins) {
    static float dt = 0, t = 0;
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, GRAVITY};

    dt = GetDeltaT(&count);
    t += dt;
    updateIMU(&ins->imu);

    ins->Accel[X] = ins->imu.acc[X];
    ins->Accel[Y] = ins->imu.acc[Y];
    ins->Accel[Z] = ins->imu.acc[Z];
    ins->Gyro[X] = ins->imu.gyro[X];
    ins->Gyro[Y] = ins->imu.gyro[Y];
    ins->Gyro[Z] = ins->imu.gyro[Z];

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    ins->atanxz = -atan2f(ins->Accel[X], ins->Accel[Z]) * 180 / PI;
    ins->atanyz = atan2f(ins->Accel[Y], ins->Accel[Z]) * 180 / PI;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(ins->Gyro[X], ins->Gyro[Y], ins->Gyro[Z], ins->Accel[X], ins->Accel[Y], ins->Accel[Z], dt);

    memcpy(ins->q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, ins->xn, ins->q);
    BodyFrameToEarthFrame(yb, ins->yn, ins->q);
    BodyFrameToEarthFrame(zb, ins->zn, ins->q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, ins->q);
    for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
    {
        ins->MotionAccel_b[i] = (ins->Accel[i] - gravity_b[i]) * dt / (ins->AccelLPF + dt) + ins->MotionAccel_b[i] * ins->AccelLPF / (ins->AccelLPF + dt);
    }
    BodyFrameToEarthFrame(ins->MotionAccel_b, ins->MotionAccel_n, ins->q); // 转换回导航系n

    ins->PitchSpeed = (ins->Pitch - ins->LastPitch)/dt; //角速度采样
    ins->YawSpeed = (ins->YawTotalAngle - ins->LastYaw)/dt;
    /*低通滤波(没滤)*/
    ins->PitchSpeed = ins->PitchSpeedLPF * ins->PitchSpeed + (1 - ins->PitchSpeedLPF) * ins->PitchLastSpeed;
    ins->YawSpeed = ins->YawSpeedLPF * ins->YawSpeed + (1 - ins->YawSpeedLPF) * ins->YawLastSpeed;
    ins->PitchLastSpeed = ins->PitchSpeed;
    ins->YawLastSpeed = ins->YawSpeed;
    ins->LastYaw = ins->YawTotalAngle;
    ins->LastPitch = ins->Pitch;
    
    // 获取最终数据
    #if SingleGyro == 0
    ins->Yaw = QEKF_INS.Yaw;
    ins->Pitch = -QEKF_INS.Roll;
    ins->Roll = QEKF_INS.Pitch;
    ins->YawTotalAngle = QEKF_INS.YawTotalAngle;
    #elif SingleGyro == 1
    ins->Yaw = QEKF_INS.Yaw;
    ins->Pitch = QEKF_INS.Pitch;
    ins->Roll = QEKF_INS.Roll;
    ins->YawTotalAngle = QEKF_INS.YawTotalAngle;
    #endif

}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / RAD_TO_ANGLE_COEF);
        cosPitch = arm_cos_f32(param->Pitch / RAD_TO_ANGLE_COEF);
        cosRoll = arm_cos_f32(param->Roll / RAD_TO_ANGLE_COEF);
        sinYaw = arm_sin_f32(param->Yaw / RAD_TO_ANGLE_COEF);
        sinPitch = arm_sin_f32(param->Pitch / RAD_TO_ANGLE_COEF);
        sinRoll = arm_sin_f32(param->Roll / RAD_TO_ANGLE_COEF);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * RAD_TO_ANGLE_COEF;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * RAD_TO_ANGLE_COEF;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * RAD_TO_ANGLE_COEF;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= RAD_TO_ANGLE_COEF;
    Pitch /= RAD_TO_ANGLE_COEF;
    Roll /= RAD_TO_ANGLE_COEF;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
