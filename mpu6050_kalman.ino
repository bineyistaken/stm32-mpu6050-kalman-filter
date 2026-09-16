#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

// State & Measurement Variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
int CalibrationNumber;
float accX, accY, accZ;
float AnglePitch, AngleRoll;

// Kalman Filter State
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2.0 * 2.0;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2.0 * 2.0;
float Kalman1DOutput[] = { 0.0, 0.0 };

// Loop Timing
const float DT = 0.004; // 250 Hz sample rate (4ms loop time)

void read_mpu6050_signals(void) {
  // Configure Digital Low-Pass Filter (DLPF) to ~10 Hz cutoff (0x1A = 0x05)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Set Accelerometer full-scale range to +/- 8g (0x1C = 0x10)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Read raw accelerometer measurements (burst read 6 bytes from 0x3B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Convert LSB to g (Scale factor: 4096 LSB/g) with offset correction
  accX = ((float)AccXLSB / 4096.0) + 0.02;
  accY = ((float)AccYLSB / 4096.0) + 0.02;
  accZ = ((float)AccZLSB / 4096.0) - 0.04;

  // Set Gyroscope full-scale range to +/- 500 deg/s (0x1B = 0x08)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Read raw gyroscope measurements (burst read 6 bytes from 0x43)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);
  
  int16_t GYRO_X = Wire.read() << 8 | Wire.read();
  int16_t GYRO_Y = Wire.read() << 8 | Wire.read();
  int16_t GYRO_Z = Wire.read() << 8 | Wire.read();

  // Convert LSB to deg/s (Scale factor: 65.5 LSB/(deg/s))
  RateRoll  = (float)GYRO_X / 65.5;
  RatePitch = (float)GYRO_Y / 65.5;
  RateYaw   = (float)GYRO_Z / 65.5;

  // Accelerometer tilt angle projection using trigonometry
  AngleRoll  = atan(accY / sqrt((accX * accX) + (accZ * accZ))) * (180.0 / 3.14159265);
  AnglePitch = atan(-accX / sqrt((accY * accY) + (accZ * accZ))) * (180.0 / 3.14159265);
}

// Discrete 1D Linear Kalman Filter
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // 1. State Prediction
  KalmanState = KalmanState + (DT * KalmanInput);
  
  // 2. Covariance Extrapolation
  KalmanUncertainty = KalmanUncertainty + (DT * DT) * (4.0 * 4.0);
  
  // 3. Kalman Gain Computation
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + (3.0 * 3.0));
  
  // 4. State Update (Correction)
  KalmanState = KalmanState + (KalmanGain * (KalmanMeasurement - KalmanState));
  
  // 5. Covariance Update
  KalmanUncertainty = (1.0 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  Serial.begin(57600);
  
  // STM32 Onboard LED (Active LOW) indicates calibration routine
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);

  // Initialize Fast-mode I2C (400 kHz)
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // Wake up MPU6050 (PWR_MGMT_1 register = 0x00)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibration: Average 2000 samples to compute stationary gyro bias offsets
  for (CalibrationNumber = 0; CalibrationNumber < 2000; CalibrationNumber++) {
    read_mpu6050_signals();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }
  RateCalibrationPitch /= 2000.0;
  RateCalibrationRoll  /= 2000.0;
  RateCalibrationYaw   /= 2000.0;

  digitalWrite(PC13, HIGH); // Calibration complete
}

void loop() {
  read_mpu6050_signals();

  // Bias cancellation
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  // 1D Kalman state fusion for Roll & Pitch
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Real-time telemetry (Roll, Pitch)
  Serial.print(KalmanAngleRoll);
  Serial.print(",");
  Serial.println(KalmanAnglePitch);

  delay(4); // Deterministic 250 Hz cycle
}