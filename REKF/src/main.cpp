#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>



 
struct __attribute__((packed)) Packet {
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
 
  for (int i = 1; i > 0; i--) {
    beep(2000, 100);
    delay(50);
    beep(3000, 100);
  }
  //Serial.println("Adafruit MPU6050 test!");

  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //Set accelerometer range
  //Serial.print("Accelerometer range set to: ");
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Set gyro range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Set bandwidth
  //Serial.print("Filter bandwidth set to: ");
  

  //Serial.println("");
  
  
  //Serial.println("Reading MPU6050 data...");
  
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

  Serial.write(0xAA);
  Serial.write((uint8_t*)&packet, sizeof(Packet));

  

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