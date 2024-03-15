#include <Wire.h>
#include <EEPROMex.h>
#define buzzerPin A1

float calA[7], calB[7], calC[7];
float ALPHA[7], BETA[7], GAMA[7];
float posA[7], posB[7], posC[7];//, sp[7];

//Gripper
float posArm = 90;
float posGrip = 90;
float posBase;// = 90;//tidak dipakai

//CoxaBodyCenterOffset
float cbcrtOfst_X[7];//coxa body centre offset X
float cbcrtOfst_Z[7];//coxa body centre offset Z
float cbcrtOfst_Y[7];
//Init Feet Position
float feetPos_x[7];//FeetPos x (wrt coxa)
float feetPos_y[7];//FeetPos y (wrt coxa)
float feetPos_z[7];//FeetPos z (wrt coxa)

//IK variable
float pos_x[7], pos_y[7], pos_z[7];
float pos_a[7], pos_b[7], pos_g[7];

float sl[7];
float Ua[7];

byte in_smoothness = 2;// kemulusan gerak
int in_nOS = in_smoothness * 12;
float perStep = in_nOS / 2;

float in_px = 0; // kiri kanan
float in_py = 0; // atas bawah
float in_pz = 0; // maju mundur
float in_pitch = 0; // putar kiri kanan
float in_roll = 0; // putar muka belakang
float in_yaw = 0; // putar kiri kanan
float in_lift = 10; // angkat kaki
byte in_gait = 1; // konfigurasi kaki
byte in_trajectory = 2; // mode langkah
float in_speed = 2; // power
float in_delay = 0; // delay antar step
float in_gama = 45; //adj coxa pojok
float in_widening = 0; //penambahan lebar oleh kaki
float in_addLength = 0; //penambahan panjang oleh kaki
float in_dirrection = 0; //arah belok

//float posArm = 40; //penambahan panjang oleh kaki
//float posGrip = 40; //arah belok

float q1 = in_nOS / float(2) - 1; //quadran 1
float q2 = in_nOS / float(2) + 1; //quadran 2
float q3 = in_nOS * 1 / float(4); //quadran 3
float q4 = in_nOS * 3 / float(4); //quadran 4

float fn1_ns = 1;
float fn2_ns = 0.75 * in_nOS;
float fn3_ns = 0.5 * in_nOS - 1;
float fn4_ns = in_nOS - 1;
float fn5_ns = 0.25 * in_nOS;
float fn6_ns = 0.5 * in_nOS + 1;

void variableLimit() {
  in_smoothness = constrain(in_smoothness, 1, 10);
  in_px = constrain(in_px, -20, 20);
  in_py = constrain(in_py, -15, 15);
  in_pz = constrain(in_pz, -20, 20);
  in_pitch = constrain(in_pitch, -10, 10);
  in_roll = constrain(in_roll, -10, 10);
  in_yaw = constrain(in_yaw, -10, 10);
  in_lift = constrain(in_lift, 0, 20);
  in_gait = constrain(in_gait, 1, 2);
  in_trajectory = constrain(in_trajectory, 1, 3);
  in_speed = constrain(in_speed, 1, 10);
  in_gama = constrain(in_gama, 0, 45);
  in_widening = constrain(in_widening, 0, 15);
  in_addLength = constrain(in_addLength, 0, 15);
  in_delay = constrain(in_delay, 0, 500);
  in_dirrection = constrain(in_dirrection, -90, 90);
}

byte u[] = {0, 0, 0, 0, 0, 0, 0};
int steep = 0;
unsigned long totalSteep = 0;
//byte inisialisasiSt = 0;
byte paternSt = 0;

float mil0, timee;
float sps;

float distancePing[5];
float distanceTof[5];

byte compasSt = 0;
int azimuth;
int fAzimuth;
byte bearing;
float startHeadings;
float headingaSadow[361];
float relativeHeadings;

float pitch;
float roll;

int jarak[3];
long countPrint = 0;

byte XYst = 0;
byte sensorSt = 0;
long beepMillis = 0;

float mapFloat(float x, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (x - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

byte pinSt = 0;
void beep() {
  if (pinSt == 0) {
    pinMode(buzzerPin, OUTPUT);
    pinSt = 1;
  }
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(100);
}
