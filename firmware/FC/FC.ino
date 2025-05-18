#include <IBusBM.h>
#include <ESP32Servo.h>
#include "KalmanAngles.h"
#include "WifiTuning.h"

// === CONFIG ===
#define REAR_RIGHT_ESC_PIN 14
#define FRONT_RIGHT_ESC_PIN 12
#define REAR_LEFT_ESC_PIN 13
#define FRONT_LEFT_ESC_PIN 15
#define IBUS_PIN 16

// === CONSTANTS ===
constexpr int LOOP_INTERVAL_US = 2500;
constexpr int MAX_MOTOR_OUTPUT = 2000;
constexpr int MIN_MOTOR_IDLE = 1180;
constexpr int MOTOR_CUTOFF = 1000;

// === STRUCTS ===
struct PIDParams
{
  float P, I, D;
};

struct PIDState
{
  float prevError = 0.0;
  float prevIterm = 0.0;
};

struct PIDController
{
  PIDParams params;
  PIDState state;
};

struct ControlSignals
{
  float roll = 0, pitch = 0, yaw = 0;
  float throttle = 0;
};

struct MotorOutputs
{
  float FR = 0, FL = 0, RR = 0, RL = 0;
};

// === GLOBALS ===
IBusBM ibus;
Servo escFR, escFL, escRR, escRL;

ControlSignals radioInput, desiredAngles, desiredRates;
ControlSignals measuredAngles, measuredRates;
ControlSignals pidOutputAngle, pidOutputRate;
MotorOutputs motorOutput;

unsigned long loopTimer = 0;

// Kalman IMU object
KalmanIMU imu;

// PID Controller Instances
PIDController pidAngleRoll{{2.0, 0.0, 0.0}};
PIDController pidAnglePitch{{2.0, 0.0, 0.0}};
PIDController pidRateRoll{{0.5, 0.3, 0.005}};
PIDController pidRatePitch{{0.55, 0.3, 0.005}};
PIDController pidRateYaw{{1.2, 0.2, 0.005}};

// === FUNCTION DECLARATIONS ===
float computePID(PIDController &controller, float error, float dt)
{
  float Pterm = controller.params.P * error;
  float Iterm = controller.state.prevIterm + controller.params.I * (error + controller.state.prevError) * dt / 2.0;
  float Dterm = controller.params.D * (error - controller.state.prevError) / dt;

  Iterm = constrain(Iterm, -400, 400);
  float output = constrain(Pterm + Iterm + Dterm, -400, 400);

  controller.state.prevError = error;
  controller.state.prevIterm = Iterm;

  return output;
}

void resetAllPID()
{
  for (PIDController *pid : {&pidAngleRoll, &pidAnglePitch, &pidRateRoll, &pidRatePitch, &pidRateYaw})
  {
    pid->state.prevError = 0;
    pid->state.prevIterm = 0;
  }
}

// === SETUP ===
void setup()
{
  Serial.begin(115200);
  delay(1500);

  escFR.attach(FRONT_RIGHT_ESC_PIN);
  escFL.attach(FRONT_LEFT_ESC_PIN);
  escRR.attach(REAR_RIGHT_ESC_PIN);
  escRL.attach(REAR_LEFT_ESC_PIN);

  Serial.println("Arming motors...");
  for (Servo *esc : {&escFR, &escFL, &escRR, &escRL})
    esc->writeMicroseconds(1000);

  Serial.println("Starting gyro calibration...");
  imu.setup(); // neue KalmanIMU-Kalibrierung

  //setup__wifi_tuning();

  Serial.println("Connecting IBUS...");
  ibus.begin(Serial2, 1, IBUS_PIN, -1);

  loopTimer = micros();
}

// === LOOP ===
void loop()
{
  static unsigned long lastLog = 0;
  unsigned long now = micros();
  float dt = (now - loopTimer) / 1e6;
  if (dt >= LOOP_INTERVAL_US / 1e6) {
    loopTimer = micros();
    // === RADIO INPUT ===
    radioInput.roll = ibus.readChannel(0);
    radioInput.pitch = ibus.readChannel(1);
    radioInput.throttle = ibus.readChannel(2);
    radioInput.yaw = ibus.readChannel(3);

    // Apply deadband for roll and pitch
    if (abs(radioInput.roll - 1500) < 11)
      radioInput.roll = 1500;
    if (abs(radioInput.pitch - 1500) < 11)
      radioInput.pitch = 1500;

    // === MAP RC INPUT TO DESIRED ANGLES / RATES ===
    desiredAngles.roll = 0.10 * (radioInput.roll - 1500);
    desiredAngles.pitch = -0.10 * (radioInput.pitch - 1500);
    desiredRates.yaw = -0.15 * (radioInput.yaw - 1500);

    // === SENSOR UPDATE ===
    imu.update();
    measuredAngles.roll = imu.getKalmanRoll(dt);
    measuredAngles.pitch = imu.getKalmanPitch(dt);
    measuredRates.roll = imu.getDroneRateRoll();
    measuredRates.pitch = imu.getDroneRatePitch();
    measuredRates.yaw = imu.getRateYaw();

    // === OUTER PID LOOP (ANGLE → RATE) ===
    float angleErrorRoll = desiredAngles.roll - measuredAngles.roll;
    float angleErrorPitch = desiredAngles.pitch - measuredAngles.pitch;
    desiredRates.roll = computePID(pidAngleRoll, angleErrorRoll, dt);
    desiredRates.pitch = computePID(pidAnglePitch, angleErrorPitch, dt);

    // === LOAD TUNED RATE PID PARAMETERS (via WiFi) ===
    pidRateRoll.params = {WifiPRateRoll, WifiIRateRoll, WifiDRateRoll};
    pidRatePitch.params = {WifiPRatePitch, WifiIRatePitch, WifiDRatePitch};
    pidRateYaw.params = {WifiPRateYaw, WifiIRateYaw, WifiDRateYaw};

    // === INNER PID LOOP (RATE → ACTUATION) ===
    float rateErrorRoll = desiredRates.roll - measuredRates.roll;
    float rateErrorPitch = desiredRates.pitch - measuredRates.pitch;
    float rateErrorYaw = desiredRates.yaw - measuredRates.yaw;

    pidOutputRate.roll = computePID(pidRateRoll, rateErrorRoll, dt);
    pidOutputRate.pitch = computePID(pidRatePitch, rateErrorPitch, dt);
    pidOutputRate.yaw = computePID(pidRateYaw, rateErrorYaw, dt);

    // === MIX PID OUTPUTS INTO MOTOR SIGNALS ===
    float t = constrain(radioInput.throttle, 1000, 1800);

    // Safety condition: below minimum throttle → stop motors & reset PID
    if (t < 1050)
    {
      resetAllPID();
      motorOutput.FR = MOTOR_CUTOFF;
      motorOutput.RR = MOTOR_CUTOFF;
      motorOutput.FL = MOTOR_CUTOFF;
      motorOutput.RL = MOTOR_CUTOFF;
    }
    else
    {
      motorOutput.FR = t - pidOutputRate.roll + pidOutputRate.pitch + pidOutputRate.yaw;
      motorOutput.RR = t - pidOutputRate.roll - pidOutputRate.pitch - pidOutputRate.yaw;
      motorOutput.FL = t + pidOutputRate.roll + pidOutputRate.pitch - pidOutputRate.yaw;
      motorOutput.RL = t + pidOutputRate.roll - pidOutputRate.pitch + pidOutputRate.yaw;

      for (float *motor : {&motorOutput.FR, &motorOutput.FL, &motorOutput.RR, &motorOutput.RL})
      {
        *motor = constrain(*motor, MIN_MOTOR_IDLE, MAX_MOTOR_OUTPUT - 1);
      }
    }

    // === ESC OUTPUT ===
    escFR.writeMicroseconds(motorOutput.FR);
    escFL.writeMicroseconds(motorOutput.FL);
    escRR.writeMicroseconds(motorOutput.RR);
    escRL.writeMicroseconds(motorOutput.RL);

    // === OPTIONAL DEBUG: LOG LOOP FREQUENCY ===
    if (millis() - lastLog > 1000)
    {
      Serial.print("Loop frequency: ");
      Serial.println(1.0 / dt);
      lastLog = millis();
    }

  }

  delay(1);
}
