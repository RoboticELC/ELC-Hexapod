#include <Servo.h>
Servo sCoxa[7];
Servo sFemur[7];
Servo sTibia[7];

Servo sReverse;
Servo sBase;
Servo sJapit;
Servo sArm;


byte testLeg = 0;
byte activeLeg[] = {1, 1, 1, 1, 1, 1, 1};

int coxaPin[] =  {0, 45, 38, 35, 6, 18, 28};
int femurPin[] = {4, 46, 39, 33, 8, 19, 31};
int tibiaPin[] = {3, 47, 40, 37, 9, 22, 30};

void activeServo() {
  int coxaPin_0[] =  {0, 0, 0, 0, 0, 0, 0};
  int femurPin_0[] = {0, 0, 0, 0, 0, 0, 0};
  int tibiaPin_0[] = {0, 0, 0, 0, 0, 0, 0};

  if (testLeg != 0) {
    for (byte fn = 1; fn <= 6; fn++) {
      sCoxa[fn].attach(coxaPin_0[fn]);
      sFemur[fn].attach(femurPin_0[fn]);
      sTibia[fn].attach(tibiaPin_0[fn]);
    }
    sCoxa[testLeg].attach(18);
    sFemur[testLeg].attach(19);
    sTibia[testLeg].attach(22);
  } else {
    openCalibrationVariable();
    for (byte fn = 1; fn <= 6; fn++) {
      sCoxa[fn].attach(coxaPin[fn]); delay(50);
      sFemur[fn].attach(femurPin[fn]); delay(50);
      sTibia[fn].attach(tibiaPin[fn]); delay(50);
      dinamicMovement(fn);
      delay(50);
    }
  }
  if (activeLeg[0] == 1) {
    delay(100);
    sArm.attach(femurPin[0]);
    delay(100);
    sJapit.attach(tibiaPin[0]);
  }
  Serial.println("activeServo ");
  //sBase.attach(coxaPin[0]);
  //sCoxa[1].write(90); sFemur[1].write(90); sTibia[1].write(90);
}

void openCalibrationVariable() {
  for (int fn = 1; fn <= 6; fn++) {
    calA[fn] = EEPROM.readFloat(4 * fn + 347);
    calB[fn] = EEPROM.readFloat(4 * fn + 371);
    calC[fn] = EEPROM.readFloat(4 * fn + 395);
  }
  Serial.println("openCalibrationVariable ");
}

void saveCalibrationVariable() {
  for (int fn = 1; fn <= 6; fn++) {
    EEPROM.writeFloat(4 * fn + 347, calA[fn]);
    EEPROM.writeFloat(4 * fn + 371, calB[fn]);
    EEPROM.writeFloat(4 * fn + 395, calB[fn]);
  }
  delay(50);
  Serial.println("saveCalibrationVariable ");
  openCalibrationVariable();
}
