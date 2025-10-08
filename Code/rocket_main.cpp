#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <stdint.h>
#include <Adafruit_BMP3XX.h>
#include <STM32SD.h>   // SD library


// ========================= TYPES =========================
struct ImuSample {
  float ax, ay, az;   // g
  float gx, gy, gz;   // deg/s
};


struct AltSample {
  float alt_m;        // meters AGL
  float press_Pa;
  float temp_C;
};


// ========================= USER SETTINGS =========================
// -------- Flight mode selection --------
enum FlightMode { MODE_BASIC = 1, MODE_LANDING = 2 };
static const FlightMode FLIGHT_MODE = MODE_LANDING;  // change to MODE_BASIC / MODE_LANDING


// -------- Safety --------
static const bool ARM_EJECTION = false;  // <<< SET TRUE ONLY FOR FIELD FLIGHT >>>


// -------- Loop timing --------
static const float LOOP_HZ = 250.0f;                   // control loop rate
static const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);


// -------- Event timing (ms) --------
static const uint32_t CANARD_DEPLOYMENT_DELAY = 3250;
static const uint32_t LANDING_BURN_DELAY = 3100;
static const uint32_t LANDING_LEGS_DELAY = 5750;


// -------- Launch detection --------
static const float LAUNCH_G_THRESH = 1.25f;            // g   (normalized)
static const uint32_t LAUNCH_DEBOUNCE_MS = 15;         // ms  above threshold


// -------- Channel Pulse Time --------
static const uint32_t LANDING_BURN_MS = 600;           // ms duration for landing motor channel
static const uint32_t LEGS_FIRE_MS = 1000;             // ms pulse to deploy legs
static const uint32_t CANARDS_FIRE_MS = 1000;          // ms pulse to deploy canards


// -------- Gimbal geometry and servo calibration --------
static const float GIMBAL_MAX_DEG_X = 5.0f;            // Pitch axis limit ±deg
static const float GIMBAL_MAX_DEG_Y = 5.0f;            // Yaw axis limit ±deg


// Sign conventions
static const int8_t SIGN_X = +1;                       // +1 or -1
static const int8_t SIGN_Y = -1;                       // +1 or -1


// Linear map: angle(deg) ↔ microseconds for each axis
static const int   SERVO_X_US_AT_NEG = 1350;           // us at -SERVO_X_DEG_AT_NEG
static const float SERVO_X_DEG_AT_NEG = -5.0f;         // deg
static const int   SERVO_X_US_AT_POS = 1950;           // us at +SERVO_X_DEG_AT_POS
static const float SERVO_X_DEG_AT_POS = +5.0f;         // deg
static const float SERVO_X_OFFSET_DEG = 0.0f;          // fine center


// Linear map: angle(deg) ↔ microseconds for each axis
static const int   SERVO_Y_US_AT_NEG = 1130;           // us at -SERVO_Y_DEG_AT_NEG
static const float SERVO_Y_DEG_AT_NEG = -5.0f;         // deg
static const int   SERVO_Y_US_AT_POS = 1370;           // us at +SERVO_Y_DEG_AT_POS
static const float SERVO_Y_DEG_AT_POS = +5.0f;         // deg
static const float SERVO_Y_OFFSET_DEG = 0.0f;          // fine center


// -------- Controllers --------
static float K_ANG_X = 0.4f;    // gimbal deg per attitude deg
static float K_ANG_Y = 0.4f;
static float K_D_X   = 0.175f;    // deg per deg/s
static float K_D_Y   = 0.175f;


static const float CF_ALPHA = 0.9f;     // higher means trust gyro more
static const float ACC_TRUST_MIN_G = 0.80f;
static const float ACC_TRUST_MAX_G = 1.20f;


// -------- SD logging --------
static const bool LOG_TO_SD = true;       // set false to disable logging


// ========================= PINMAP =========================
// Servos
static const int SERVO_PITCH_PIN = A0;    // Pitch axis (X)
static const int SERVO_YAW_PIN   = A1;    // Yaw axis (Y)


// LEDs
static const int LED1_PIN = PC13;         // status/ready YELLOW
static const int LED2_PIN = PD5;          // boost/flight RED
static const int LED3_PIN = PD6;          // events GREEN


// Ejection channels
static const int EJECT_CANARDS_PIN   = PC1;    // canards
static const int EJECT_LEGS_PIN    = PA9;      // landing legs
static const int EJECT_LANDING_PIN = PC0;      // landing motor ignite


// SD Card (STM32SD pin mapping)
static inline void configureSdPins() {
  SD.setDx(PC8, PC9, PC10, PC11);
  SD.setCMD(PD2);
  SD.setCK(PC12);
}


// ========================= GLOBALS =========================
Servo servoX, servoY;


// state
static bool sd_ok = false;
static File logFile;


static bool launched = false;
static uint32_t t_launch_ms = 0;


static bool landingIgnited = false;
static uint32_t landingIgniteOff_ms = 0;
static bool legsDeployed = false;
static uint32_t legsOff_ms = 0;
static bool canardsDeployed = false;
static uint32_t canardsOff_ms = 0;


// estimators
static float ground_alt = 0.0f;
static float alt_filt = 0.0f;
static float vel_filt = 0.0f;


static float roll_deg  = 0.0f;
static float pitch_deg = 0.0f;


// commanded mount angles
static float cmd_x_deg = 0.0f;
static float cmd_y_deg = 0.0f;


// loop timing
static uint32_t t_loop_us = 0;


// ========================= SENSOR WRAPPERS =========================
bool sensorsBegin();
void readAccelGyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz); // accel in g, gyro in dps
bool readAltPressureTemp(float &alt_m, float &press_Pa, float &temp_C);


/* -------------------- BMI088 (bit-banged I2C) -------------------- */
#define PIN_SCL PA5   // P1-2
#define PIN_SDA PA7   // P1-4
#define PIN_ADR PA6   // P1-3


static inline void scl_release(){ pinMode(PIN_SCL, INPUT_PULLUP); }
static inline void scl_drive_lo(){ pinMode(PIN_SCL, OUTPUT); digitalWrite(PIN_SCL, LOW); }
static inline void sda_release(){ pinMode(PIN_SDA, INPUT_PULLUP); }
static inline void sda_drive_lo(){ pinMode(PIN_SDA, OUTPUT); digitalWrite(PIN_SDA, LOW); }
static inline int  sda_read(){ return digitalRead(PIN_SDA); }
static inline void i2c_delay(){ delayMicroseconds(4); }  // ~100 kHz


static void i2c_start(){ sda_release(); scl_release(); i2c_delay(); sda_drive_lo(); i2c_delay(); scl_drive_lo(); }
static void i2c_stop(){  sda_drive_lo(); i2c_delay(); scl_release(); i2c_delay(); sda_release(); i2c_delay(); }
static bool i2c_writeByte(uint8_t b){
  for(int i=7;i>=0;--i){ ((b>>i)&1) ? sda_release() : sda_drive_lo(); i2c_delay(); scl_release(); i2c_delay(); scl_drive_lo(); }
  sda_release(); i2c_delay(); scl_release(); bool nack = sda_read(); i2c_delay(); scl_drive_lo(); return !nack;
}
static uint8_t i2c_readByte(bool ack){
  uint8_t v=0; sda_release();
  for(int i=7;i>=0;--i){ i2c_delay(); scl_release(); v |= (sda_read()<<i); i2c_delay(); scl_drive_lo(); }
  if(ack) sda_drive_lo(); else sda_release(); i2c_delay(); scl_release(); i2c_delay(); scl_drive_lo(); sda_release(); return v;
}
static bool i2c_writeReg(uint8_t a7, uint8_t r, uint8_t v){
  i2c_start(); if(!i2c_writeByte((a7<<1)|0)) { i2c_stop(); return false; }
  if(!i2c_writeByte(r))                      { i2c_stop(); return false; }
  if(!i2c_writeByte(v))                      { i2c_stop(); return false; }
  i2c_stop(); delayMicroseconds(10); return true;
}
static bool i2c_readRegs(uint8_t a7, uint8_t r, uint8_t* buf, int n){
  i2c_start(); if(!i2c_writeByte((a7<<1)|0)) { i2c_stop(); return false; }
  if(!i2c_writeByte(r))                      { i2c_stop(); return false; }
  i2c_start(); if(!i2c_writeByte((a7<<1)|1)) { i2c_stop(); return false; }
  for(int i=0;i<n;i++) buf[i]=i2c_readByte(i<n-1);
  i2c_stop(); return true;
}


/* BMI088 – gyro regs */
static const uint8_t REG_GYR_CHIP_ID   = 0x00; // 0x0F
static const uint8_t REG_RATE_X_LSB    = 0x02; // 0x07
static const uint8_t REG_GYR_RANGE     = 0x0F;
static const uint8_t REG_GYR_BANDWIDTH = 0x10;
static const uint8_t REG_GYR_LPM1      = 0x11;
static const uint8_t REG_GYR_SOFTRESET = 0x14;
static uint8_t gyroAddr = 0x68;
static float dpsPerLsb(uint8_t r){
  switch(r & 0x07){
    case 0x00: return 1.0f/16.384f;   // +-2000
    case 0x01: return 1.0f/32.768f;   // +-1000
    case 0x02: return 1.0f/65.536f;   // +-500
    case 0x03: return 1.0f/131.072f;  // +-250
    case 0x04: return 1.0f/262.144f;  // +-125
    default:   return 1.0f/16.384f;
  }
}


/* BMI088 – accel regs */
static const uint8_t REG_ACC_CHIP_ID   = 0x00; // 0x1E
static const uint8_t REG_ACC_X_LSB     = 0x12;
static const uint8_t REG_ACC_CONF      = 0x40;
static const uint8_t REG_ACC_RANGE     = 0x41; // 0:+-3g,1:+-6g,2:+-12g,3:+-24g
static const uint8_t REG_ACC_PWR_CONF  = 0x7C;
static const uint8_t REG_ACC_PWR_CTRL  = 0x7D; // 0x04 = accel enable
static const uint8_t REG_ACC_SOFTRESET = 0x7E;
static uint8_t accelAddr = 0x18;


/* -------------------- BMP390 -------------------- */
Adafruit_BMP3XX bmp390;


// keep ADR pin consistent with detected gyro address
static inline void setAdrPinForGyro(){
  pinMode(PIN_ADR, OUTPUT);
  if (gyroAddr == 0x69) digitalWrite(PIN_ADR, HIGH); else digitalWrite(PIN_ADR, LOW);
}


// ========================= UTILS =========================
static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }


static int angleToUs(float deg, int us_neg, float deg_neg, int us_pos, float deg_pos, float offset_deg) {
  float d0 = deg_neg;
  float u0 = (float)us_neg;
  float d1 = deg_pos;
  float u1 = (float)us_pos;
  float m  = (u1 - u0) / (d1 - d0);   // us per deg
  float b  = u0 - m * d0;
  float u  = m * (deg + offset_deg) + b;
  int ui   = (int)roundf(u);
  // hard clamp typical servo range
  if (ui < 800) ui = 800; if (ui > 2200) ui = 2200;
  return ui;
}


static inline int angleToUsX(float deg) {
  return angleToUs(deg * SIGN_X, SERVO_X_US_AT_NEG, SERVO_X_DEG_AT_NEG,
                   SERVO_X_US_AT_POS, SERVO_X_DEG_AT_POS, SERVO_X_OFFSET_DEG);
}
static inline int angleToUsY(float deg) {
  return angleToUs(deg * SIGN_Y, SERVO_Y_US_AT_NEG, SERVO_Y_DEG_AT_NEG,
                   SERVO_Y_US_AT_POS, SERVO_Y_DEG_AT_POS, SERVO_Y_OFFSET_DEG);

  deg *= SIGN_Y;
  return 2.4f * ((deg + 6.04167f) * (deg + 6.04167f)) + 1427.39583f;
}


static void writeServos_deg(float x_deg, float y_deg) {
  cmd_x_deg = clampf(x_deg, -GIMBAL_MAX_DEG_X, +GIMBAL_MAX_DEG_X);
  cmd_y_deg = clampf(y_deg, -GIMBAL_MAX_DEG_Y, +GIMBAL_MAX_DEG_Y);
  servoX.writeMicroseconds(angleToUsX(cmd_x_deg));
  servoY.writeMicroseconds(angleToUsY(cmd_y_deg));
}


// ========================= LED HELPERS =========================
static void ledsOff() { digitalWrite(LED1_PIN, LOW); digitalWrite(LED2_PIN, LOW); digitalWrite(LED3_PIN, LOW); }
static void bootBlink() {
  for (int i = 0; i < 6; ++i) { digitalWrite(LED3_PIN, (i & 1)); delay(80); }
  ledsOff();
}


// ========================= EJECTION HELPERS =========================
static void safeEjections() {
  digitalWrite(EJECT_CANARDS_PIN, LOW);
  digitalWrite(EJECT_LEGS_PIN, LOW);
  digitalWrite(EJECT_LANDING_PIN, LOW);
}


static void firePinTimed(int pin, uint32_t ms) {
  if (!ARM_EJECTION) return;   // safety lockout
  digitalWrite(pin, HIGH);
  uint32_t off_at = millis() + ms;
  if (pin == EJECT_LANDING_PIN) landingIgniteOff_ms = off_at;
  if (pin == EJECT_LEGS_PIN)    legsOff_ms = off_at;
  if (pin == EJECT_CANARDS_PIN)  canardsOff_ms = off_at;
}


// ========================= SD LOGGING =========================
static char logName[16];


static void openLog() {
  if (!LOG_TO_SD) return;
  for (int i = 1; i <= 99999; ++i) {
    snprintf(logName, sizeof(logName), "/TVC%05d.CSV", i);
    if (!SD.exists(logName)) {
      logFile = SD.open(logName, FILE_WRITE);
      sd_ok = logFile ? true : false;
      if (sd_ok) {
        logFile.println("t_ms,ax,ay,az,gx,gy,gz,alt,vel,roll,pitch,cmdX,cmdY,mode,flags");
        logFile.flush();
      }
      break;
    }
  }
}


static inline void logRow(uint32_t t_ms, const ImuSample &imu, float alt, float vel, float rolld, float pitchd) {
  if (!sd_ok) return;
  uint8_t flags = 0;
  if (launched) flags |= 0x01;
  if (landingIgnited) flags |= 0x04;
  if (legsDeployed) flags |= 0x08;
  logFile.print(t_ms); logFile.print(',');
  logFile.print(imu.ax, 5); logFile.print(',');
  logFile.print(imu.ay, 5); logFile.print(',');
  logFile.print(imu.az, 5); logFile.print(',');
  logFile.print(imu.gx, 4); logFile.print(',');
  logFile.print(imu.gy, 4); logFile.print(',');
  logFile.print(imu.gz, 4); logFile.print(',');
  logFile.print(alt, 3); logFile.print(',');
  logFile.print(vel, 3); logFile.print(',');
  logFile.print(rolld, 2); logFile.print(',');
  logFile.print(pitchd, 2); logFile.print(',');
  logFile.print(cmd_x_deg, 2); logFile.print(',');
  logFile.print(cmd_y_deg, 2); logFile.print(',');
  logFile.print((int)FLIGHT_MODE); logFile.print(',');
  logFile.println(flags);
  if ((t_ms & 0x1F) == 0) logFile.flush();
}


// ========================= SETUP =========================
void setup() {
  pinMode(LED1_PIN, OUTPUT); pinMode(LED2_PIN, OUTPUT); pinMode(LED3_PIN, OUTPUT);
  pinMode(EJECT_CANARDS_PIN, OUTPUT); pinMode(EJECT_LEGS_PIN, OUTPUT); pinMode(EJECT_LANDING_PIN, OUTPUT);
  safeEjections();
  ledsOff();
  bootBlink();


  Serial.begin(115200);
  delay(500);
  Serial.println("Serial open.");


  configureSdPins();
 
  if (!SD.begin()) {
    // SD init failed
    Serial.println("SD initialization failed.");
    for (;;) { digitalWrite(LED2_PIN, !digitalRead(LED2_PIN)); delay(1000); }
  }


  servoX.attach(SERVO_PITCH_PIN, 800, 2200);
  servoY.attach(SERVO_YAW_PIN,   800, 2200);
  writeServos_deg(0, 0);


  if (!sensorsBegin()) {
    // Flash red if sensors fail
    Serial.println("Sensors failed.");
    for (;;) { digitalWrite(LED2_PIN, !digitalRead(LED2_PIN)); delay(1000); }
  }
  delay(250); // Wait for the sensors to initialize


  // Zero altitude from few samples
  float sum = 0; int n = 0; AltSample as; float p, t;
  // Purge the bad readings out of the BMP390
  float z;
  for (int i = 0; i < 10; ++i) {
    readAltPressureTemp(z, p, t);
  }
  for (int i = 0; i < 100; ++i) {
    float a; if (readAltPressureTemp(a, p, t)) { sum += a; ++n; }
    delay(10);
  }
  ground_alt = (n > 0) ? (sum / n) : 0.0f;
  alt_filt = 0.0f; // AGL
  vel_filt = 0.0f;


  openLog();


  // Ready pattern: LED1 slow blink for 2s
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) { digitalWrite(LED3_PIN, (millis() >> 8) & 1); delay(10); }
  ledsOff();


  t_loop_us = micros();


  Serial.println("Setup complete.");
}


// ========================= LOOP =========================
void loop() {
  // pacing
  uint32_t now = micros();
  if ((now - t_loop_us) < LOOP_US) return;
  float dt = (now - t_loop_us) * 1e-6f;
  t_loop_us = now;


  // --- read sensors ---
  ImuSample imu; readAccelGyro(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
  float alt_raw, p, tc;
  if (readAltPressureTemp(alt_raw, p, tc)) {
    float alt_agl = alt_raw - ground_alt;
    // low-pass alt and differentiate to velocity
    const float ALT_LP = 0.10f;   // tune
    float alt_prev = alt_filt;
    alt_filt = (1.0f - ALT_LP) * alt_filt + ALT_LP * alt_agl;
    float vel_raw = (alt_filt - alt_prev) / dt;
    const float VEL_LP = 0.18f;
    vel_filt = (1.0f - VEL_LP) * vel_filt + VEL_LP * vel_raw;
  }


  // --- launch detect ---
  float a_norm = sqrtf(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
  static uint32_t a_over_t0 = 0;
  if (!launched && FLIGHT_MODE != MODE_BASIC) {
    if (a_norm > LAUNCH_G_THRESH) {
      if (a_over_t0 == 0) a_over_t0 = millis();
      if (millis() - a_over_t0 >= LAUNCH_DEBOUNCE_MS) {
        launched = true; t_launch_ms = millis()-LAUNCH_DEBOUNCE_MS;
        digitalWrite(LED3_PIN, HIGH); // indicate flight
      }
    } else {
      a_over_t0 = 0;
    }
  }


  // --- attitude estimate ---
  // integrate gyro
  roll_deg  += imu.gz * dt + 0.085 * dt;   // gx about X (roll) -- bias correction term here
  pitch_deg += imu.gy * dt + 0.135 * dt;   // gy about Y (pitch) -- bias correction term here


  // accel-based tilt (only trust pre-launch)
  float roll_acc  = radians(0.0f);
  float pitch_acc = radians(0.0f);

  // Use standard tilt from accel.
  // all values are tested and accurate despite axes looking wrong
  roll_acc  = atan2f(sqrtf(imu.ax*imu.ax + imu.az*imu.az), imu.ay) * 57.29578f - 90;
  pitch_acc = -atan2f(sqrtf(imu.ax*imu.ax + imu.ay*imu.ay), imu.az) * 57.29578f + 90;

  // don't add the accelerometer readings if in launch or under abnormal g loads (does weird things in flight)
  if (!launched && a_norm > ACC_TRUST_MIN_G && a_norm < ACC_TRUST_MAX_G) {
    roll_deg  = CF_ALPHA * roll_deg  + (1.0f - CF_ALPHA) * roll_acc;
    pitch_deg = CF_ALPHA * pitch_deg + (1.0f - CF_ALPHA) * pitch_acc;
  }


  if (!launched && FLIGHT_MODE != MODE_BASIC) {
    // pre-launch: blink LED1
    digitalWrite(LED1_PIN, (millis() >> 9) & 1);
  }


  // --- control law ---
  float cmdX = 0.0f, cmdY = 0.0f;

  // Angle hold + rate damping
  const float setX = 0.0f;  // target rocket angle
  const float setY = 0.0f;
  float errX = setX - roll_deg;
  float errY = setY - pitch_deg;
  cmdX = K_ANG_X * errX - K_D_X * imu.gz;
  cmdY = K_ANG_Y * errY - K_D_Y * imu.gy;


  // clamp and send
  writeServos_deg(clampf(cmdY, -GIMBAL_MAX_DEG_X, +GIMBAL_MAX_DEG_X),
                  clampf(cmdX, -GIMBAL_MAX_DEG_Y, +GIMBAL_MAX_DEG_Y));


  // fire all timed channels                
  if (FLIGHT_MODE == MODE_LANDING && launched && ARM_EJECTION) {
    uint32_t now_ms = millis();

    // deploy canards when delay passed
    if (!canardsDeployed && (now_ms - t_launch_ms >= CANARD_DEPLOYMENT_DELAY)) {
      canardsDeployed = true;
      firePinTimed(EJECT_CANARDS_PIN, CANARDS_FIRE_MS);
    }
    // ignite landing motor when delay passed
    if (!landingIgnited && (now_ms - t_launch_ms >= LANDING_BURN_DELAY)) {
      landingIgnited = true;
      firePinTimed(EJECT_LANDING_PIN, LANDING_BURN_MS);
    }
    // deploy legs when delay passed
    if (!legsDeployed && (now_ms - t_launch_ms >= LANDING_LEGS_DELAY)) {
      legsDeployed = true;
      firePinTimed(EJECT_LEGS_PIN, LEGS_FIRE_MS);
    }
  }


  // turn off timed channels
  if (landingIgnited && landingIgniteOff_ms && millis() > landingIgniteOff_ms) {
    digitalWrite(EJECT_LANDING_PIN, LOW); landingIgniteOff_ms = 0;
  }
  if (legsDeployed && legsOff_ms && millis() > legsOff_ms) {
    digitalWrite(EJECT_LEGS_PIN, LOW); legsOff_ms = 0;
  }
  if (canardsDeployed && canardsOff_ms && millis() > canardsOff_ms) {
    digitalWrite(EJECT_CANARDS_PIN, LOW); canardsOff_ms = 0;
  }


  // LED2 blinks during flight, LED1 off
  if (launched) {
    // flight indicator
    digitalWrite(LED2_PIN, (millis() >> 8) & 1); // simple blink to indicate flight
  }


  // log
  logRow(millis(), imu, alt_filt, vel_filt, roll_deg, pitch_deg);
}


// ========================= SENSOR IMPL (BMI088 bit-banged + Adafruit BMP390) =========================


bool sensorsBegin() {
  // ----- BMI088 address select pin and bus idle -----
  pinMode(PIN_ADR, OUTPUT); digitalWrite(PIN_ADR, LOW);
  sda_release(); scl_release();


  // Probe gyro (0x68/0x69)
  for (int tryHigh=0; tryHigh<2; ++tryHigh){
    digitalWrite(PIN_ADR, tryHigh ? HIGH : LOW);
    uint8_t id=0xFF; (void)i2c_readRegs(tryHigh?0x69:0x68, REG_GYR_CHIP_ID, &id, 1);
    if (id == 0x0F){ gyroAddr = tryHigh?0x69:0x68; break; }
  }
  uint8_t gid=0xFF; (void)i2c_readRegs(gyroAddr, REG_GYR_CHIP_ID, &gid, 1);
  if (gid != 0x0F) return false;


  // Probe accel (0x18/0x19)
  for (uint8_t a : { (uint8_t)0x18, (uint8_t)0x19 }){
    uint8_t id=0xFF; (void)i2c_readRegs(a, REG_ACC_CHIP_ID, &id, 1);
    if (id == 0x1E){ accelAddr = a; break; }
  }
  uint8_t aid=0xFF; (void)i2c_readRegs(accelAddr, REG_ACC_CHIP_ID, &aid, 1);
  if (aid != 0x1E) return false;


  // Lock ADR to address
  setAdrPinForGyro();


  // Gyro init
  i2c_writeReg(gyroAddr, REG_GYR_SOFTRESET, 0xB6); delay(30);
  i2c_writeReg(gyroAddr, REG_GYR_LPM1,      0x00);
  i2c_writeReg(gyroAddr, REG_GYR_RANGE,     0x00);  // ±2000 dps
  i2c_writeReg(gyroAddr, REG_GYR_BANDWIDTH, 0x04);
  delay(5); // wait


  // Accel init
  i2c_writeReg(accelAddr, REG_ACC_SOFTRESET, 0xB6); delay(2);
  i2c_writeReg(accelAddr, REG_ACC_PWR_CONF,  0x00); delay(5);
  i2c_writeReg(accelAddr, REG_ACC_PWR_CTRL,  0x04); delay(50);
  i2c_writeReg(accelAddr, REG_ACC_CONF,      0xA8); // 100 Hz, normal filter
  i2c_writeReg(accelAddr, REG_ACC_RANGE,     0x01); // +-6 g


  // ----- BMP390 on I2C1 -----
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(400000);


  bool ok = bmp390.begin_I2C(0x76, &Wire);
  if (!ok) ok = bmp390.begin_I2C(0x77, &Wire);
  if (!ok) return false;


  bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp390.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp390.setOutputDataRate(BMP3_ODR_50_HZ);


  return true;
}


void readAccelGyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  // Check ADR pin matches the selected gyro address
  setAdrPinForGyro();
  sda_release(); scl_release(); // ensure bus idles high


  // ----- BMI088 gyro -----
  uint8_t graw[6];
  if(!i2c_readRegs(gyroAddr, REG_RATE_X_LSB, graw, 6)) { ax=ay=0; az=1; gx=gy=gz=0; return; }
  int16_t grx = (int16_t)((uint16_t)graw[1]<<8 | graw[0]);
  int16_t gry = (int16_t)((uint16_t)graw[3]<<8 | graw[2]);
  int16_t grz = (int16_t)((uint16_t)graw[5]<<8 | graw[4]);
  uint8_t grange=0; (void)i2c_readRegs(gyroAddr, REG_GYR_RANGE, &grange, 1);
  float kdps = dpsPerLsb(grange);


  // ----- BMI088 accel -----
  uint8_t araw[6];
  if(!i2c_readRegs(accelAddr, REG_ACC_X_LSB, araw, 6)) { ax=ay=0; az=1; gx=grx*kdps; gy=gry*kdps; gz=grz*kdps; return; }
  int16_t arx = (int16_t)((uint16_t)araw[1]<<8 | araw[0]);
  int16_t ary = (int16_t)((uint16_t)araw[3]<<8 | araw[2]);
  int16_t arz = (int16_t)((uint16_t)araw[5]<<8 | araw[4]);
  uint8_t arange=0; (void)i2c_readRegs(accelAddr, REG_ACC_RANGE, &arange, 1);
  float kmg = (1000.0f * (1 << ((arange&3)+1)) * 1.5f) / 32768.0f; // mg/LSB


  ax = (arx * kmg) / 1000.0f;
  ay = (ary * kmg) / 1000.0f;
  az = (arz * kmg) / 1000.0f;


  gx = grx * kdps;
  gy = gry * kdps;
  gz = grz * kdps;
}


bool readAltPressureTemp(float &alt_m, float &press_Pa, float &temp_C) {
  if (!bmp390.performReading()) return false;
  temp_C   = bmp390.temperature;      // Celsius
  press_Pa = bmp390.pressure;        // Pascals
  // Baro altitude from sea level; setup() zeros to AGL using ground_alt
  const float P0 = 101325.0f; // Pa
  float ratio = press_Pa / P0; if (ratio <= 0) ratio = 1e-6f;
  alt_m = 44330.0f * (1.0f - powf(ratio, 0.1903f));
  return true;
}
