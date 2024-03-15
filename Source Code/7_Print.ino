void printSPS() {
  if (printSt == 1) {
    timee = millis() - mil0;
    sps = float(1000) / timee;
    Serial.print("sps ");
    Serial.print(sps);
    Serial.print(", distance ");
    Serial.print(sps * in_pz * 2 / 10);
    Serial.print(", RunSt ");
    Serial.println(runSt);
    mil0 = millis();
  }
}

void print_CalABC(byte fn) {
  Serial.print("calA[" + String(fn) + "]"); Serial.print(calA[fn]); Serial.print("\t");
  Serial.print("calB[" + String(fn) + "]"); Serial.print(calB[fn]); Serial.print("\t");
  Serial.print("calC[" + String(fn) + "]"); Serial.print(calC[fn]); //Serial.print("\t");
  //Serial.print("t_agl[" + String(fn) + "]"); Serial.print(t_angle[fn]);
  Serial.println("");
}

void print_New_pos(byte fn) {
  Serial.print("new_pos_x[" + String(fn) + "]"); Serial.print(new_pos_x[fn]); Serial.print("\t");
  Serial.print("new_pos_y[" + String(fn) + "]"); Serial.print(new_pos_y[fn]); Serial.print("\t");
  Serial.print("new_pos_z[" + String(fn) + "]"); Serial.print(new_pos_z[fn]); Serial.println("");
}

void print_IKWS(byte fn) {
  Serial.print("IKWS[" + String(fn) + "]"); Serial.print(IKWS[fn]); Serial.print("\t");
  Serial.print("IKA1[" + String(fn) + "]"); Serial.print(IKA1[fn]); Serial.print("\t");
  Serial.print("IKA2[" + String(fn) + "]"); Serial.print(IKA2[fn]); Serial.print("\t");
  Serial.print("t_agl[" + String(fn) + "]"); Serial.print(t_angle[fn]); Serial.println("");
}

void print_servoAction(byte fn) {
  Serial.print("sCx[" + String(fn) + "] "); Serial.print(posA[fn]); Serial.print("\t");
  Serial.print("sFm[" + String(fn) + "] "); Serial.print(posB[fn]); Serial.print("\t");
  Serial.print("sTb[" + String(fn) + "] "); Serial.print(posC[fn]); Serial.println("");
}

void print_Inisialisasi() {
  for (int fn = 1; fn <= 6; fn++) {
    Serial.print("feetPos_x[" + String(fn) + "]"); Serial.print(feetPos_x[fn]); Serial.print("\t");
    Serial.print("feetPos_z[" + String(fn) + "]"); Serial.print(feetPos_z[fn]); Serial.print("\t");
    Serial.print("feetPos_y[" + String(fn) + "]"); Serial.print(feetPos_y[fn]); Serial.println();
    //Serial.println();
  }
}

void print_calIK(byte fn) {
  Serial.print("GAMA[" + String(fn) + "]"); Serial.print(GAMA[fn]); Serial.print("\t");
  Serial.print("ALPHA[" + String(fn) + "]"); Serial.print(ALPHA[fn]); Serial.print("\t");
  Serial.print("BETA[" + String(fn) + "]"); Serial.print(BETA[fn]); Serial.println("");
}
