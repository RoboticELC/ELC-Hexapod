String dt[30];
String dataIn;
boolean parsing = false;
int i;

void daser() {
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    dataIn += inChar;
    if (inChar == '\n') {
      parsing = true;
    }
  }
  if (parsing) {
    parsingData();
    parsing = false;
    dataIn = "";
  }
}

void parsingData() {
  int j = 0;
  Serial.print("data masuk : "); Serial.print(dataIn); Serial.print("\n");

  dt[j] = "";
  for (i = 0; i < dataIn.length(); i++) {
    if ((dataIn[i] == '#') || (dataIn[i] == ',')) {
      j++;
      dt[j] = "";
    } else {
      dt[j] = dt[j] + dataIn[i];
    }
  }

  if (dt[0] == "run") {
    runSt = dt[1].toInt();
    Serial.print("runSt : = ");
    Serial.println(runSt);
    //activeServo();
  }

  if (dt[0] == "testLeg") {
    testLeg = dt[1].toInt();
    Serial.print("testLeg : = ");
    Serial.println(testLeg);
    activeServo();
  } else
    //===================in_Variable
    if (dt[0] == "x") {
      in_px = dt[1].toFloat();
      Serial.print("in_px : = ");
      Serial.println(in_px);
    } else if (dt[0] == "y") {
      in_py = dt[1].toFloat();
      Serial.print("in_py : = ");
      Serial.println(in_py);
    } else if (dt[0] == "z") {
      in_pz = dt[1].toFloat();
      Serial.print("in_pz : = ");
      Serial.println(in_pz);
    } else if (dt[0] == "pitch") {
      in_pitch = dt[1].toFloat();
      Serial.print("in_pitch : = ");
      Serial.println(in_pitch);
    } else if (dt[0] == "roll") {
      in_roll = dt[1].toFloat();
      Serial.print("in_roll : = ");
      Serial.println(in_roll);
    } else if (dt[0] == "yaw") {
      in_yaw = dt[1].toFloat();
      Serial.print("in_yaw : = ");
      Serial.println(in_yaw);
    } else if (dt[0] == "lift") {
      in_lift = dt[1].toFloat();
      Serial.print("in_lift : = ");
      Serial.println(in_lift);
    } else if (dt[0] == "gait") {
      in_gait = dt[1].toFloat();
      Serial.print("in_gait : = ");
      Serial.println(in_gait);
    } else if (dt[0] == "trj") {
      in_trajectory = dt[1].toFloat();
      Serial.print("in_trajectory : = ");
      Serial.println(in_trajectory);
    } else if (dt[0] == "s") {
      in_speed = dt[1].toFloat();
      Serial.print("in_speed : = ");
      Serial.println(in_speed);
    } else if (dt[0] == "S") {
      in_smoothness = dt[1].toInt();
      Serial.print("in_smoothness : = ");
      Serial.println(in_smoothness);

      smoothness(in_smoothness);
      //paternSt = 0;
    } else if (dt[0] == "gama") {
      in_gama = dt[1].toFloat();
      Serial.print("in_gama : = ");
      Serial.println(in_gama);
//      inisialisasiSt = 0;
    } else if (dt[0] == "wide") {
      in_widening = dt[1].toFloat();
      Serial.print("in_widening : = ");
      Serial.println(in_widening);
    } else if (dt[0] == "length") {
      in_addLength = dt[1].toFloat();
      Serial.print("in_addLength : = ");
      Serial.println(in_addLength);
    } else if (dt[0] == "delay") {
      in_delay = dt[1].toFloat();
      Serial.print("in_delay : = ");
      Serial.println(in_delay);
    } else
      //==========Tune
      if (dt[0] == "C") {
        int fn = dt[1].toInt();
        calA[fn] = dt[2].toFloat();
        Serial.print("calA[" + String(fn) + "] ");
        Serial.println(calA[fn]);
        EEPROM.writeFloat(4 * fn + 347, calA[fn]);
      } else if (dt[0] == "F") {
        int fn = dt[1].toInt();
        calB[fn] = dt[2].toFloat();
        Serial.print("calB[" + String(fn) + "] ");
        Serial.println(calB[fn]);
        EEPROM.writeFloat(4 * fn + 371, calB[fn]);
      } else if (dt[0] == "T") {
        int fn = dt[1].toInt();
        calC[fn] = dt[2].toFloat();
        Serial.print("calC[" + String(fn) + "] ");
        Serial.println(calC[fn]);
        EEPROM.writeFloat(4 * fn + 395, calC[fn]);
      }
  variableLimit();
}

void EEPROM_ARR(int d) {
  int saveDataSt = EEPROM.readInt(0);
  if (saveDataSt != d) {
    saveCalibrationVariable();
    saveDataSt = d;
    EEPROM.writeInt(0, saveDataSt);
    Serial.println("Saving Data");
  } else {
    openCalibrationVariable();
  }
}
