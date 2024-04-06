# Project Overview
This readme provides an overview of how to the KRI Hexapod Program.

# Pre-requisites
To begin, please install the library > Library Hexapod 2024, the package includes:
- NewPing
- Adafruit VL53L0X (ToF/Distance sensor, file: 4_Sensor)
- Arduino EEPROMEx-master: data storage for reading and writing basic types, structs, strings, arrays
- Simple MPU 6050 Sensor
- Servo
- QMC5883L Compass Meter
- RemoteXY (location mapping for x and y axis w/ RemoteXY library)

### Hardware yang disetting rujuk dari ```pinMode```
- buzzer > 1_variable
- push button > Hexapod2024_Kentdry_Tangerang 
- ToF sensor > 4_Sensor 
- Baterai > 4_Sensor

# Full Sequence ```Hexapod2024_Kentdry_Tangerang``` 

### Initial Setup: ```variables initialized```
- import from 1_variable
- int runSt = 0; 
- byte printSt = 1; 
- float jarakKiri, jarakKanan, jarakDepan, jarakBelakang, jarakGrip, jarakKorban; 
- int pushButtonPin = 36; 
- byte pushSt, tofSt, pingSt = 0;

### Initial Setup: ```void setup()```
- jalur komunikasi 2: serial (setting hardware sensor, servo, dll), serial3 (bluetooth)
- establish bluetooth communication (setupXY) > ```RemoteXY```
- pertama jalan kalibrasi, simpan derajat masing-masing servo di penyimpanan data kedua, ketiga, etc > ```EEPROM_ARR(2)``` dalam bytes
- pin SCL, SDA, GPI01, XSHUT > ```setupTof()``` dari hardware VL53L0X
- ```INACTIVE``` pin SCL, SDA, XDA, XCL, ADO, INT > ```setupGyro()``` dari hardware MPU-6050
- ```INACTIVE``` pin SCL, SDA, DRDY ```setupCompass()``` dari hardware QMC5883L
- inverse kinematics u/ Servo ```initialisai();```
- aktivasi servo coxa, femur, tibia 18 servo ```activeServo();```
- signal beep to user lewat buzzer ```beep();```

### Loop: ```void loop()```
- aktifkan ```pushButton()```
- aktifkan ```demo_01()```
- ```INACTIVE``` aktifkan ```demo_02()```
- ```INACTIVE``` aktifkan ```demo_03()```
- ```INACTIVE``` aktifkan ```daser()```
- aktifkan ```loopXY()```
- aktifkan ```RUNNING()```
