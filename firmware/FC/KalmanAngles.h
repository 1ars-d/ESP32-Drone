#ifndef KALMAN_ANGLES_H
#define KALMAN_ANGLES_H

#include <Wire.h>

class KalmanIMU
{
public:
    KalmanIMU();

    void setup();
    void update();

    float getKalmanPitch(float dt);
    float getKalmanRoll(float dt);

    float getAccX() const;
    float getAccY() const;
    float getAccZ() const;

    float getDroneRateRoll() const;
    float getDroneRatePitch() const;
    float getRateYaw() const;
    float getRateRoll() const;
    float getRatePitch() const;

private:
    void calibrate();
    void kalman1D(float &state, float &uncertainty, float rate, float measuredAngle, float dt);

    float accX, accY, accZ;
    float rateRoll, ratePitch, rateYaw;

    float rateCalibRoll = 0.63, rateCalibPitch = -0.52, rateCalibYaw = 0.06;
    float accXCalib = 0.01, accYCalib = -0.01, accZCalib = 0.09;

    float kalmanAngleRoll = 0, kalmanUncertaintyRoll = 4;
    float kalmanAnglePitch = 0, kalmanUncertaintyPitch = 4;

    float angleRoll, anglePitch;

    static constexpr uint8_t MPU_ADDR = 0x68;
};

#endif
