
#include <Wire.h>
#include <Mpu6050.h>

Mpu6050 gyro_sensor;
Mpu6050Data data;

byte gyroSt = 1;

void setupGyro() {
  gyro_sensor = Mpu6050();
  
  if (gyro_sensor.init()) {
    Serial.println("Mpu6050 Connected!");
  } else {
    Serial.println("Failed to connect to Mpu6050.");
  }
  gyroSt = 1;
  Serial.println("setupGyro ");
  delay (4000);
}

void Baca_gyro () {
  
  data = gyro_sensor.readData();
  Serial.println("---------------------------------------");
  Serial.print("Acceleration (m/s^2): X=");    // percepatan linear x,y,z
  Serial.print(data.acceleration.x);          // Kondisi diam Acceleration (m/s^2): X=0.81 Y=-0.26 Z=10.48 
  Serial.print(" Y=");                        // Kondisi belok kanan Acceleration (m/s^2): X=0.51 Y=1.87 Z=9.08
  Serial.print(data.acceleration.y);          // Kondisi jalan lurus Acceleration (m/s^2): X=1.04 Y=-0.05 Z=12.41
  Serial.print(" Z=");                      
  Serial.println(data.acceleration.z);
  Serial.print("Gyroscope (°/s): X=");          // kecepatan sudut (seberapa cepat modul berputar di sekitar (x,y,z)
  Serial.print(data.gyroscope.x);               // Kondisi diam Gyroscope (°/s): X=-4.18 Y=-1.09 Z=-1.82
  Serial.print(" Y=");                          // Kondisi belok kanan Gyroscope (°/s): X=-22.34 Y=-2.77 Z=-6.78
  Serial.print(data.gyroscope.y);               // Kondisi jalan lurus Gyroscope (°/s): X=-4.23 Y=3.66 Z=0.39
  Serial.print(" Z="); 
  Serial.println(data.gyroscope.z);
//  Serial.print("Temperature (°C): "); 
//  Serial.println(data.temperature);
  delay(500);
  
}

void getGyro() {
  
  if (gyroSt == 1) {
    data = gyro_sensor.readData();
    float pitch = data.acceleration.y * 10;
    float roll = data.acceleration.x * 10;
    float accelPitch = data.gyroscope.y;
    float accelRoll = data.gyroscope.x;

    Serial.print ("pitch = ");
    Serial.print (pitch);
    Serial.print (" | roll = ");
    Serial.println (roll);
  }

}
