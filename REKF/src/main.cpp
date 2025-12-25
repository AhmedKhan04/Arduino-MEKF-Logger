#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>



 
struct Packet {
  uint32_t t;
  float ax, ay, az;
  float gx, gy, gz;
};











Adafruit_MPU6050 mpu;

constexpr uint8_t BUZZER_PIN = 9;

void beep(uint16_t freq, uint16_t duration_ms) {
  tone(BUZZER_PIN, freq, duration_ms);
  delay(duration_ms);
  noTone(BUZZER_PIN);
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  Serial.println("Adafruit MPU6050 test!");

  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //Set accelerometer range
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Set gyro range
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Set bandwidth
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  for (int i = 5; i > 0; i--) {
    beep(2000, 100);
    delay(50);
    beep(3000, 100);
  }
  
  Serial.println("Reading MPU6050 data...");
  
  delay(100);
}

void loop() {
  static uint32_t lastBeepTime = 0; 

  Packet packet;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  packet.t = millis();
  packet.ax = a.acceleration.x;
  packet.ay = a.acceleration.y;
  packet.az = a.acceleration.z;
  packet.gx = g.gyro.x;
  packet.gy = g.gyro.y;
  packet.gz = g.gyro.z;

  Serial.write((uint8_t*)&packet, sizeof(Packet));

  if (millis() - lastBeepTime >= 30000) {
    lastBeepTime = millis();
    beep(500, 100);
    delay(50);
    beep(1000, 100);
  }

  delayMicroseconds(5000); // 200 Hz

  /* Get new sensor events with the readings */
  /*
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  /* Print out the values 
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
  */
}