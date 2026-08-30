#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68

#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define ACCEL_XOUT_H 0x3B

const float ACCEL_SCALE = 8192.0;   
const float GYRO_SCALE  = 65.5;     
const float G_TO_MS2    = 9.80665;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(PWR_MGMT_1, 0x00);    
  delay(100);
  mpuWrite(ACCEL_CONFIG, 0x08);  
  mpuWrite(GYRO_CONFIG, 0x08);   
}

void mpuRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); 
  Wire.read();   
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = (rawAx / ACCEL_SCALE) * G_TO_MS2;
  ay = (rawAy / ACCEL_SCALE) * G_TO_MS2;
  az = (rawAz / ACCEL_SCALE) * G_TO_MS2;
  gx = rawGx / GYRO_SCALE;
  gy = rawGy / GYRO_SCALE;
  gz = rawGz / GYRO_SCALE;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== MPU-6500 calibration-only sketch starting ===");

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("FAILED — nothing responded at 0x68. Check wiring/power.");
    while (1) delay(1000);
  }

  mpuInit();
  Serial.println("SUCCESS — sensor initialized.");

  Serial.println();
  Serial.println("Hold the robot exactly upright and still now.");
  Serial.println("Calibration starts in 3 seconds...");
  delay(3000);

  float sum = 0;
  float ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 200; i++) {
    mpuRead(ax, ay, az, gx, gy, gz);
    sum += atan2(ax, az) * 180.0 / PI;
    delay(3);
  }
  float angleOffset = sum / 200.0;

  Serial.println();
  Serial.print("DONE. angleOffset = ");
  Serial.println(angleOffset, 4);
  Serial.println("Copy this number down for use in the main sketch.");
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  mpuRead(ax, ay, az, gx, gy, gz);
  float angle = atan2(ax, az) * 180.0 / PI;

  Serial.print("Live accelerometer angle: ");
  Serial.println(angle, 2);
  delay(200);
}