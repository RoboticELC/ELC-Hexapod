/*
   Hexapod_Kinematik_H04_03a July 2021
   Hexapod_Kinematik_H04_2022 Mei-2022
   Hexapod_Kinematik_H05a_2022 Dec-2022

   Inspiration from OscarLiang.com
   https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/

   Hexapod_Kinematik_H06_2023 April 2023
   Hexapod_Kinematik_H06_2023 Edited Jan 2024
   by Saiful Jamaan, P. hD
*/

#include "1_Variable.h"

int runSt = 0;
byte printSt = 1;
float jarakKiri, jarakKanan, jarakDepan, jarakBelakang, jarakGrip, jarakKorban;
int pushButtonPin = 36;
byte pushSt = 0;
byte tofSt = 0;
byte pingSt = 0;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.println("Hexapod KRSRI 2024");
  pinMode(pushButtonPin, INPUT_PULLUP);
  setupXY();//remote by Android
  EEPROM_ARR(2);
  setupTof();
  //setupGyro();
  //setupCompass();
  initialisai();
  activeServo();
  beep(); beep();
  Serial.println("Setup Ok");
  Serial.println();
}

//void loopTest() {
void loop1() {
  if (tofSt == 1) {
    Serial.print("jarak_L "); Serial.println(getDistanceTof(0));
    Serial.print("jarak_R "); Serial.println(getDistanceTof(2));
    Serial.print("jarak_B "); Serial.println(getDistancePing(0));
    Serial.print("jarak_F "); Serial.println(getDistancePing(1));
    Serial.print("jarak_G "); Serial.println(getDistancePing(4));
  }

  if (compasSt == 1) {
    getRelativeHeadings();
    Serial.print("relativeHeadings "); Serial.println(relativeHeadings);
    //testBacaArah();
  }
  Serial.println();
  delay(500);
}

void loop() {
  pushButton();
  demo_01();
  //demo_02();
  //demo_03();
  //daser();// data Serial
  loopXY();// setting by Android
  RUNNING();
}

void RUNNING() {
  readSensor();
  variableLimit();
  if (steep < in_nOS) {
    steep++;
    delay(in_delay);
  } else {
    steep = 1;
    delay(in_delay);
    totalSteep++;
  }
  for (byte fn = 1; fn <= 6; fn++) {
    dinamicMovement(fn);
  }
}

float rRH = 0;
int per = 1;
void readSensor() {
  if (sensorSt == 1) {
    if (steep == 3) {
      jarakKorban = getDistancePing(4);
    }
    if (steep == 6) {
      jarakKanan = getDistanceTof(0);
    }
    if (steep == 6) {
      jarakDepan = getDistancePing(1);
    }
    if (steep == 9) {
      jarakKiri = getDistanceTof(2);
    }
    if (steep == 12) {
      jarakBelakang = getDistancePing(0);
    }

    if (compasSt == 1) {
      if (steep > 1) {
        getRelativeHeadings();
        rRH += relativeHeadings;
        per++;
      } else {
        teganganBaterai();
        relativeHeadings = (rRH / per);// + 15;
        rRH = 0;
        per = 1;
        Serial.print("rHs ");
        Serial.println(relativeHeadings, 1);
      }
    }
  }
}

void starValue() {
  in_dirrection = 0;//arah belok
  in_px = 0; // kiri kanan
  in_py = 5; // tinggi body (ground clearence)
  in_pz = 0; // maju mundur
  in_pitch = 0; //angkat kiri kanan
  in_roll = 0; //angkat depan belakang
  in_yaw = 0; // putar kiri kanan
  in_lift = 12; //angkat kaki saat melangkah
  in_speed = 5; // kecepatan
  in_delay = 0;
  in_gama = 45;
}

void stopValue() {
  in_px = 0; // kiri kanan
  in_py = 0; // atas bawah
  in_pz = 0; // maju mundur
  in_pitch = 0; // putar kiri kanan
  in_roll = 0; // putar muka belakang
  in_yaw = 0; // putar kiri kanan
  in_lift = 0; // angkat kaki
}

long oldMillis, countStart;//
void pushButton() {
  if (digitalRead(pushButtonPin) == LOW) {
    if (pushSt == 0) {
      if (runSt == 0) {
        runSt = 10;
        setupCompass();
        countStart = 2;
      } else {
        runSt = 0;
        beep(); beep();
      }
      pushSt = 1;
    }
    delay(100);
  } else pushSt = 0;

  if (runSt == 10) {
    if (millis() - oldMillis >= 1000) {
      oldMillis = millis();
      countStart--;
      beep();
      if (pushSt == 0) {
        if (countStart == 0) runSt = 1;
      } else {
        if (countStart == 0) {
          stopValue();
          for (byte fn = 1; fn <= 6; fn++) {
            dinamicMovement(fn);
          }
          runSt = 0;
          steep = 0;
        }
      }
      Serial.print("runSt ");
      Serial.println(runSt);
    }
  }
}
