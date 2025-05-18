#include "KalmanAngles.h"
#include <math.h>

KalmanIMU::KalmanIMU() {}

void KalmanIMU::setup()
{
    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    // Wake up MPU-6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Set Digital Low Pass Filter (~10Hz)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    // Set Accelerometer range to ±8g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Set Gyroscope range to ±500°/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
}

void KalmanIMU::calibrate()
{
    float sumRateRoll = 0, sumRatePitch = 0, sumRateYaw = 0;
    float sumAccX = 0, sumAccY = 0, sumAccZ = 0;

    for (int i = 0; i < 2000; ++i)
    {
        update();

        sumRateRoll += rateRoll;
        sumRatePitch += ratePitch;
        sumRateYaw += rateYaw;

        sumAccX += accX;
        sumAccY += accY;
        sumAccZ += accZ - 1;

        delay(1);
    }

    rateCalibRoll = sumRateRoll / 2000;
    rateCalibPitch = sumRatePitch / 2000;
    rateCalibYaw = sumRateYaw / 2000;

    accXCalib = sumAccX / 2000;
    accYCalib = sumAccY / 2000;
    accZCalib = sumAccZ / 2000;
}

void KalmanIMU::update()
{
    // Acc Setup
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
    int16_t accXRaw = Wire.read() << 8 | Wire.read();
    int16_t accYRaw = Wire.read() << 8 | Wire.read();
    int16_t accZRaw = Wire.read() << 8 | Wire.read();

    accX = accXRaw / 4096.0 - accXCalib;
    accY = accYRaw / 4096.0 - accYCalib;
    accZ = accZRaw / 4096.0 - accZCalib;

    // Gyro Setup
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
    int16_t gyroX = Wire.read() << 8 | Wire.read();
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    int16_t gyroZ = Wire.read() << 8 | Wire.read();

    rateRoll = gyroX / 65.5f - rateCalibRoll;
    ratePitch = gyroY / 65.5f - rateCalibPitch;
    rateYaw = gyroZ / 65.5f - rateCalibYaw;

    // Angle Calculation
    angleRoll = atan2(accY, sqrt(accX * accX + accZ * accZ)) * (180.0 / M_PI);
    anglePitch = -atan2(accX, sqrt(accY * accY + accZ * accZ)) * (180.0 / M_PI);
}

void KalmanIMU::kalman1D(float &state, float &uncertainty, float rate, float measuredAngle, float dt)
{
    state += dt * rate;
    uncertainty += dt * dt * 16;                  // Q: process noise
    float kalmanGain = uncertainty / (uncertainty + 9); // R: measurement noise
    state += kalmanGain * (measuredAngle - state);
    uncertainty *= (1 - kalmanGain);
}

float KalmanIMU::getKalmanPitch(float dt)
{
    kalman1D(kalmanAngleRoll, kalmanUncertaintyRoll, rateRoll, angleRoll, dt);
    return kalmanAngleRoll;
}

float KalmanIMU::getKalmanRoll(float dt)
{
    kalman1D(kalmanAnglePitch, kalmanUncertaintyPitch, ratePitch, anglePitch, dt);
    return kalmanAnglePitch;
}

float KalmanIMU::getAccX() const { return accX; }
float KalmanIMU::getAccY() const { return accY; }
float KalmanIMU::getAccZ() const { return accZ; }

float KalmanIMU::getDroneRateRoll() const { return ratePitch; }
float KalmanIMU::getDroneRatePitch() const { return rateRoll; }
float KalmanIMU::getRateYaw() const { return rateYaw; }
float KalmanIMU::getRateRoll() const { return rateRoll; }
float KalmanIMU::getRatePitch() const { return ratePitch; }
