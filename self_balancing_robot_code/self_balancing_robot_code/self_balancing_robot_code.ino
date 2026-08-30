#include <Wire.h>

// ============================================================
// PIN MAP
// ============================================================

// MPU-6500 I2C
#define SDA_PIN 8
#define SCL_PIN 9

// DRV8833
#define AIN1 4
#define AIN2 5
#define BIN1 6
#define BIN2 7
#define STBY 15

// Encoders
#define ENC_L_C1 16
#define ENC_L_C2 17
#define ENC_R_C1 18
#define ENC_R_C2 21

// ============================================================
// CALIBRATION SETTINGS
// ============================================================


#define CALIBRATE_ONLY 0

#define HARDCODED_OFFSET 3.9010

// ============================================================
// MPU-6500 REGISTERS
// ============================================================

#define MPU_ADDR 0x68

#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B

const float ACCEL_SCALE = 8192.0;


const float GYRO_SCALE = 65.5;

const float G_TO_MS2 = 9.80665;

// ============================================================
// STATE
// ============================================================

float angle = 0.0;
float angleOffset = 0.0;

float gyroYOffset = 0.0;

unsigned long lastTime = 0;

// ============================================================
// CONTROLLER
// ============================================================


float Kp = 45;
float Ki = 0.0;
float Kd = 0.0;

float setpoint = 0.0;

float integral = 0.0;

const float FALL_LIMIT = 45.0;

// ============================================================
// ENCODERS
// ============================================================

volatile long encLCount = 0;
volatile long encRCount = 0;

void IRAM_ATTR encL_ISR() {
  encLCount += digitalRead(ENC_L_C2) ? 1 : -1;
}

void IRAM_ATTR encR_ISR() {
  encRCount += digitalRead(ENC_R_C2) ? 1 : -1;
}

// ============================================================
// MPU LOW-LEVEL FUNCTIONS
// ============================================================

bool mpuWrite(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool mpuReadRegister(uint8_t reg, uint8_t &value) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDR, 1) != 1) {
    return false;
  }

  value = Wire.read();

  return true;
}

// ============================================================
// MPU INITIALIZATION
// ============================================================

bool mpuInit() {

  if (!mpuWrite(PWR_MGMT_1, 0x00)) {
    return false;
  }

  delay(100);

  mpuWrite(CONFIG, 0x04);

  mpuWrite(GYRO_CONFIG, 0x08);

  mpuWrite(ACCEL_CONFIG, 0x08);

  delay(50);

  return true;
}

// ============================================================
// READ MPU DATA
// ============================================================

bool mpuRead(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
) {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  int received = Wire.requestFrom(MPU_ADDR, 14);

  if (received != 14) {
    return false;
  }

  int16_t rawAx =
    ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawAy =
    ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawAz =
    ((int16_t)Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  int16_t rawGx =
    ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawGy =
    ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t rawGz =
    ((int16_t)Wire.read() << 8) | Wire.read();

  ax = ((float)rawAx / ACCEL_SCALE) * G_TO_MS2;
  ay = ((float)rawAy / ACCEL_SCALE) * G_TO_MS2;
  az = ((float)rawAz / ACCEL_SCALE) * G_TO_MS2;

  gx = (float)rawGx / GYRO_SCALE;
  gy = (float)rawGy / GYRO_SCALE;
  gz = (float)rawGz / GYRO_SCALE;

  return true;
}

// ============================================================
// ANGLE CALIBRATION
// ============================================================

void calibrateAngle() {

  Serial.println();
  Serial.println("Hold robot PERFECTLY upright and still.");
  Serial.println("Starting angle calibration...");
  delay(2000);

  float sum = 0.0;
  int validSamples = 0;

  float ax, ay, az;
  float gx, gy, gz;

  for (int i = 0; i < 500; i++) {

    if (mpuRead(ax, ay, az, gx, gy, gz)) {

      float accAngle =
        atan2(ax, az) * 180.0 / PI;

      sum += accAngle;

      validSamples++;
    }

    delay(3);
  }

  if (validSamples > 0) {

    angleOffset =
      sum / validSamples;

    angle = angleOffset;

  } else {

    Serial.println("ERROR: no valid MPU samples.");
  }
}

// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro() {

  Serial.println();
  Serial.println("Calibrating gyro...");
  Serial.println("DO NOT MOVE THE ROBOT.");

  delay(1000);

  float sumY = 0.0;

  int validSamples = 0;

  float ax, ay, az;
  float gx, gy, gz;

  for (int i = 0; i < 1000; i++) {

    if (mpuRead(ax, ay, az, gx, gy, gz)) {

      sumY += gy;

      validSamples++;
    }

    delay(2);
  }

  if (validSamples > 0) {

    gyroYOffset =
      sumY / validSamples;

    Serial.print("Gyro Y offset = ");
    Serial.println(gyroYOffset, 5);

  } else {

    Serial.println("ERROR: gyro calibration failed.");
  }
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void driveMotors(float speed) {

  bool forward = speed >= 0;

  int pwm =
    constrain((int)fabs(speed), 0, 255);

  // LEFT MOTOR

  analogWrite(
    AIN1,
    forward ? pwm : 0
  );

  analogWrite(
    AIN2,
    forward ? 0 : pwm
  );

  // RIGHT MOTOR

  analogWrite(
    BIN1,
    forward ? pwm : 0
  );

  analogWrite(
    BIN2,
    forward ? 0 : pwm
  );
}

void stopMotors() {

  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);

  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(2000);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" SELF BALANCING ROBOT STARTING");
  Serial.println("====================================");

  // ----------------------------------------------------------
  // Motors
  // ----------------------------------------------------------

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  stopMotors();

  digitalWrite(STBY, HIGH);

  // ----------------------------------------------------------
  // Encoders
  // ----------------------------------------------------------

  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);

  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_L_C1),
    encL_ISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_R_C1),
    encR_ISR,
    RISING
  );

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(400000);

  Serial.println("I2C started.");

  // ----------------------------------------------------------
  // Find MPU
  // ----------------------------------------------------------

  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() != 0) {

    Serial.println();
    Serial.println("ERROR: MPU not found at 0x68.");
    Serial.println("Check SDA, SCL, VCC and GND.");

    stopMotors();

    while (true) {

      Serial.println("MPU NOT FOUND");
      delay(1000);
    }
  }

  Serial.println("MPU detected.");

  // ----------------------------------------------------------
  // Check WHO_AM_I
  // ----------------------------------------------------------

  uint8_t whoAmI = 0;

  if (mpuReadRegister(WHO_AM_I, whoAmI)) {

    Serial.print("WHO_AM_I = 0x");

    if (whoAmI < 0x10) {
      Serial.print("0");
    }

    Serial.println(whoAmI, HEX);

  } else {

    Serial.println("Unable to read WHO_AM_I.");
  }

  // ----------------------------------------------------------
  // Configure MPU
  // ----------------------------------------------------------

  if (!mpuInit()) {

    Serial.println("MPU initialization failed.");

    stopMotors();

    while (true) {
      delay(1000);
    }
  }

  Serial.println("MPU initialized.");

  // ----------------------------------------------------------
  // Calibration
  // ----------------------------------------------------------

#if CALIBRATE_ONLY

  calibrateAngle();

  Serial.println();
  Serial.println("============================");

  Serial.print("angleOffset = ");
  Serial.println(angleOffset, 4);

  Serial.println("============================");

  Serial.println();
  Serial.println("Copy this number into:");
  Serial.println("HARDCODED_OFFSET");

  Serial.println();
  Serial.println("Then set:");
  Serial.println("CALIBRATE_ONLY 0");

  stopMotors();

  while (true) {
    delay(1000);
  }

#else

  angleOffset =
    HARDCODED_OFFSET;

  angle =
    angleOffset;

  Serial.print("Using hardcoded angle offset: ");
  Serial.println(angleOffset, 4);

#endif

  calibrateGyro();

  Serial.println();
  Serial.println("Controller starting.");

  delay(500);

  lastTime = micros();
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  float ax, ay, az;
  float gx, gy, gz;

  // ----------------------------------------------------------
  // Read sensor
  // ----------------------------------------------------------

  if (!mpuRead(
        ax, ay, az,
        gx, gy, gz
      )) {

    stopMotors();

    Serial.println("MPU READ FAILED");

    delay(10);

    return;
  }

  // ----------------------------------------------------------
  // Calculate dt
  // ----------------------------------------------------------

  unsigned long now =
    micros();

  float dt =
    (now - lastTime)
    / 1000000.0;

  lastTime = now;

  if (dt <= 0.0 || dt > 0.05) {

    stopMotors();

    return;
  }

  // ----------------------------------------------------------
  // Accelerometer angle
  // ----------------------------------------------------------

  float accAngle =
    atan2(ax, az)
    * 180.0
    / PI;

  // ----------------------------------------------------------
  // Gyro rate
  // ----------------------------------------------------------

  float gyroRate =
    gy - gyroYOffset;


  // ----------------------------------------------------------
  // Complementary filter
  // ----------------------------------------------------------

  angle =
      0.98
      * (angle + gyroRate * dt)

    + 0.02
      * accAngle;

  float tilt =
    angle - angleOffset;

  // ----------------------------------------------------------
  // Fall safety
  // ----------------------------------------------------------

  if (fabs(tilt) > FALL_LIMIT) {

    stopMotors();

    integral = 0.0;

    return;
  }

  // ----------------------------------------------------------
  // Controller
  // ----------------------------------------------------------

  float error =
    setpoint - tilt;

  integral +=
    error * dt;

  integral =
    constrain(
      integral,
      -50.0,
      50.0
    );


  float output =
      Kp * error
    + Ki * integral
    + Kd * gyroRate;

  output =
    constrain(
      output,
      -255.0,
      255.0
    );

  // ----------------------------------------------------------
  // Motor output
  // ----------------------------------------------------------

  driveMotors(output);

  // ----------------------------------------------------------
  // Optional debug output
  // ----------------------------------------------------------

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 100) {

    lastPrint = millis();

    Serial.print("tilt: ");
    Serial.print(tilt, 2);

    Serial.print("  gyro: ");
    Serial.print(gyroRate, 2);

    Serial.print("  output: ");
    Serial.print(output, 1);

    Serial.print("  encL: ");
    Serial.print(encLCount);

    Serial.print("  encR: ");
    Serial.println(encRCount);
  }
}
