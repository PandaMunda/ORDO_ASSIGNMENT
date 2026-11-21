#include <Wire.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

const float ACCEL_SCALE = 8192.0f; 
const float GYRO_SCALE  = 65.5f; 

float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float base_temp = 25.0f;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// Timing & Filter Tuning
unsigned long last_time_us = 0;
const float ALPHA = 0.98f; 
const float GYRO_DEADZONE = 0.08f; 

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &temp, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  ax   = (Wire.read() << 8) | Wire.read();
  ay   = (Wire.read() << 8) | Wire.read();
  az   = (Wire.read() << 8) | Wire.read();
  temp = (Wire.read() << 8) | Wire.read();
  gx   = (Wire.read() << 8) | Wire.read();
  gy   = (Wire.read() << 8) | Wire.read();
  gz   = (Wire.read() << 8) | Wire.read();
}

void calibrateSensor(int samples = 1000) {
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  long sum_temp = 0;

  Serial.println("Calibrating... Keep MPU-6050 flat and completely still.");
  delay(500);

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, temp, gx, gy, gz;
    readRawData(ax, ay, az, temp, gx, gy, gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += (az - (int16_t)ACCEL_SCALE); 
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    sum_temp += temp;

    delay(2);
  }

  ax_offset = (float)sum_ax / samples;
  ay_offset = (float)sum_ay / samples;
  az_offset = (float)sum_az / samples;
  gx_offset = (float)sum_gx / samples;
  gy_offset = (float)sum_gy / samples;
  gz_offset = (float)sum_gz / samples;
  base_temp = ((float)sum_temp / samples) / 340.0f + 36.53f;

  Serial.println("Calibration complete!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); 

 
  writeRegister(REG_PWR_MGMT_1, 0x01);
  delay(100);


  writeRegister(REG_CONFIG, 0x03);

 
  writeRegister(REG_SMPLRT_DIV, 0x04);


  writeRegister(REG_GYRO_CONFIG, 0x08);
  writeRegister(REG_ACCEL_CONFIG, 0x08);

  calibrateSensor(1000);

  int16_t ax, ay, az, temp, gx, gy, gz;
  readRawData(ax, ay, az, temp, gx, gy, gz);
  float fax = (ax - ax_offset) / ACCEL_SCALE;
  float fay = (ay - ay_offset) / ACCEL_SCALE;
  float faz = (az - az_offset) / ACCEL_SCALE;

  roll  = atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0f / PI;
  pitch = atan2(-fax, sqrt(fay * fay + faz * faz)) * 180.0f / PI;

  last_time_us = micros();
}

void loop() {
  int16_t raw_ax, raw_ay, raw_az, raw_temp, raw_gx, raw_gy, raw_gz;
  readRawData(raw_ax, raw_ay, raw_az, raw_temp, raw_gx, raw_gy, raw_gz);

  unsigned long now_us = micros();
  float dt = (now_us - last_time_us) / 1000000.0f;
  last_time_us = now_us;

  float current_temp = (raw_temp / 340.0f) + 36.53f;
  float temp_delta = current_temp - base_temp;

  float gx_comp = ((raw_gx - gx_offset) / GYRO_SCALE) - (temp_delta * 0.03f);
  float gy_comp = ((raw_gy - gy_offset) / GYRO_SCALE) - (temp_delta * 0.03f);
  float gz_comp = ((raw_gz - gz_offset) / GYRO_SCALE) - (temp_delta * 0.03f);

  float ax_comp = ((raw_ax - ax_offset) / ACCEL_SCALE) - (temp_delta * 0.0005f);
  float ay_comp = ((raw_ay - ay_offset) / ACCEL_SCALE) - (temp_delta * 0.0005f);
  float az_comp = ((raw_az - az_offset) / ACCEL_SCALE) - (temp_delta * 0.0005f);

  if (fabs(gx_comp) < GYRO_DEADZONE) gx_comp = 0.0f;
  if (fabs(gy_comp) < GYRO_DEADZONE) gy_comp = 0.0f;
  if (fabs(gz_comp) < GYRO_DEADZONE) gz_comp = 0.0f;

  float accel_roll  = atan2(ay_comp, sqrt(ax_comp * ax_comp + az_comp * az_comp)) * 180.0f / PI;
  float accel_pitch = atan2(-ax_comp, sqrt(ay_comp * ay_comp + az_comp * az_comp)) * 180.0f / PI;

  float accel_mag = sqrt(ax_comp * ax_comp + ay_comp * ay_comp + az_comp * az_comp);
  float alpha = (fabs(accel_mag - 1.0f) > 0.15f) ? 0.998f : ALPHA;

  roll  = alpha * (roll  + gx_comp * dt) + (1.0f - alpha) * accel_roll;
  pitch = alpha * (pitch + gy_comp * dt) + (1.0f - alpha) * accel_pitch;
  yaw  += gz_comp * dt; 

  static unsigned long last_print = 0;
  if (millis() - last_print >= 20) {
    last_print = millis();
    Serial.printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Temp: %.1fC\n", roll, pitch, yaw, current_temp);
  }
}