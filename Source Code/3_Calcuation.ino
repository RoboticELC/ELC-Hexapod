//=========LEG VARIABLE
float cx = 30;  //coxa
float fm = 40;  //femur
float tb = 70;  //tibia

//=========BODY VARIABLE
float fbW = 60; // 60 front and back body Wide
float cW = 90; // 120 centre body Wide
float bL = 150; // body Lengh
float teta = 68.19;

byte olds = 0;
void smoothness(byte s) { //smoothness dr pergerakan, menentukan kecepatan putar servo
  if (olds != s) {
    in_nOS = s * 12;
    perStep = in_nOS / 2;

    q1 = in_nOS / float(2) - 1; //quadran 1
    q2 = in_nOS / float(2) + 1; //quadran 2
    q3 = in_nOS * 1 / float(4); //quadran 3
    q4 = in_nOS * 3 / float(4); //quadran 3

    fn1_ns = 1;
    fn2_ns = 0.75 * in_nOS;
    fn3_ns = 0.5 * in_nOS - 1;
    fn4_ns = in_nOS - 1;
    fn5_ns = 0.25 * in_nOS;
    fn6_ns = 0.5 * in_nOS + 1;
    olds = s;
  }
}
float oldGama = 0;
void initialisai() { //inisialisasi posisi2 awal dr servo dan robot
  smoothness(in_smoothness);
  if (oldGama != in_gama) {
    feetPos_x[1] = cos((teta - in_gama) / float(180) * PI) * (cx + fm);
    feetPos_x[2] = cos(0 / float(180) * PI)  * (cx + fm);
    feetPos_x[3] = cos((teta - in_gama) / float(180) * PI)  * (cx + fm);
    feetPos_x[4] = -cos((teta - in_gama) / float(180) * PI) * (cx + fm);
    feetPos_x[5] = -cos(0 / float(180) * PI) * (cx + fm);
    feetPos_x[6] = -cos((teta - in_gama) / float(180) * PI) * (cx + fm);

    feetPos_z[1] = sin((teta - in_gama) / float(180) * PI) * (cx + fm);
    feetPos_z[2] = sin(0 / float(180) * PI) * (cx + fm);
    feetPos_z[3] = -sin((teta - in_gama) / float(180) * PI) * (cx + fm);
    feetPos_z[4] = -sin((teta - in_gama) / float(180) * PI) * (cx + fm);
    feetPos_z[5] = -sin(0 / float(180) * PI) * (cx + fm);
    feetPos_z[6] = sin((teta - in_gama) / float(180) * PI) * (cx + fm);

    for (int fn = 1; fn <= 6; fn++) {
      feetPos_y[fn] = tb;
    }
    cbcrtOfst_X[1] = fbW / 2;
    cbcrtOfst_X[2] = cW / 2;
    cbcrtOfst_X[3] = fbW / 2;
    cbcrtOfst_X[4] = -fbW / 2;
    cbcrtOfst_X[5] = -cW / 2;
    cbcrtOfst_X[6] = -fbW / 2;

    cbcrtOfst_Z[1] = bL / 2;
    cbcrtOfst_Z[2] = 0;
    cbcrtOfst_Z[3] = -bL / 2;
    cbcrtOfst_Z[4] = -bL / 2;
    cbcrtOfst_Z[5] = 0;
    cbcrtOfst_Z[6] = bL / 2;
    Serial.println("Inisialisasi ");
   oldGama = in_gama;
  }
}

void staticMovement(int fn) { //mungkin pergerakan tanpa pindah lokasi (muter badan, hadap kanan/kiri)
  initialisai();
  pos_x[fn] = in_px;
  pos_y[fn] = in_py;
  pos_z[fn] = in_pz;
  pos_a[fn] = in_pitch;
  pos_b[fn] = in_roll;
  pos_g[fn] = in_yaw;
  cal_IK(pos_x[fn], pos_y[fn],  pos_z[fn], pos_a[fn], pos_b[fn], pos_g[fn], fn);
}

void dinamicMovement(byte fn) { //kebalikan static, mungkin jalan kemana
  initialisai();
  gaitCal(fn);  trajectoryCal(fn);
  if (fn < 4) {
    pos_x[fn] = in_widening + (in_px / perStep * sl[fn]);
  } else {
    pos_x[fn] = -in_widening + (in_px / perStep * sl[fn]);
  }

  if (fn == 1) pos_z[fn] = in_addLength + in_pz / perStep * sl[fn];
  if (fn == 2) pos_z[fn] = in_pz / perStep * sl[fn];
  if (fn == 3) pos_z[fn] = -in_addLength + in_pz / perStep * sl[fn];

  //if (fn == 1) pos_z[fn] = -in_addLength + in_pz / perStep * sl[fn];
  //if (fn == 2) pos_z[fn] = -in_pz / perStep * sl[fn];
  //if (fn == 3) pos_z[fn] = in_addLength + in_pz / perStep * sl[fn];

  if (fn == 4) pos_z[fn] = -in_addLength + in_pz / perStep * sl[fn];
  if (fn == 5) pos_z[fn] = in_pz / perStep * sl[fn];
  if (fn == 6) pos_z[fn] = in_addLength + in_pz / perStep * sl[fn];

  pos_y[fn] = in_py + Ua[fn];//up
  pos_a[fn] = in_roll;// / perStep * sl[fn];
  pos_b[fn] = in_pitch;// / perStep * sl[fn];
  pos_g[fn] = in_yaw / perStep * sl[fn];

  cal_IK(pos_x[fn], pos_y[fn],  pos_z[fn], pos_a[fn], pos_b[fn], pos_g[fn], fn);
}

float IKWS[7], IKA1[7], IKA2[7];
float t_angle[7];
float cxRunAngle[] = {0, 60, 0, -60, 60, 0, -60};
float total_X[7], total_Z[7], total_Y[7];
float sin_rot_X[7], sin_rot_Z[7], sin_rot_Y[7];
float cos_rot_X[7], cos_rot_Z[7], cos_rot_Y[7];
float bodyIK_x[7], bodyIK_z[7], bodyIK_y[7];
float new_pos_x[7], new_pos_y[7], new_pos_z[7];
float cxFeetDistance[7];

void cal_IK(float px, float py, float pz, float Pitch, float Roll, float yaw, int fn) { //rumus
  //=========Body IK============
  pos_x[fn] = px;
  pos_y[fn] = py;
  pos_z[fn] = pz;

  total_X[fn] = feetPos_x[fn] + cbcrtOfst_X[fn] + pos_x[fn];
  total_Z[fn] = feetPos_z[fn] + cbcrtOfst_Z[fn] + pos_z[fn];
  total_Y[fn] = feetPos_y[fn];
  //total_Y[fn] = feetPos_y[fn] + cbcrtOfst_Y[fn] + pos_y[fn];

  sin_rot_X[fn] = sin(Pitch / float(180) * PI);
  sin_rot_Z[fn] = sin(Roll / float(180) * PI);
  sin_rot_Y[fn] = sin(yaw / float(180) * PI);

  cos_rot_X[fn] = cos(Pitch / float(180) * PI);
  cos_rot_Z[fn] = cos(Roll / float(180) * PI);
  cos_rot_Y[fn] = cos(yaw / float(180) * PI);

  bodyIK_x[fn] = (total_X[fn] * cos_rot_Z[fn] * cos_rot_Y[fn])
                 - (total_Z[fn] * cos_rot_Z[fn] * sin_rot_Y[fn])
                 - (total_Y[fn] * sin_rot_Z[fn]) - total_X[fn];

  bodyIK_z[fn] = ((total_X[fn] * cos_rot_X[fn] * sin_rot_Y[fn])
                  + (total_X[fn] * cos_rot_Y[fn] * sin_rot_Z[fn] * sin_rot_X[fn])
                  + (total_Z[fn] * cos_rot_Y[fn] * cos_rot_X[fn])
                  - (total_Z[fn] * sin_rot_X[fn] * sin_rot_Z[fn] * sin_rot_X[fn])
                  - (total_Y[fn] * cos_rot_Z[fn] * sin_rot_X[fn])) - total_Z[fn];

  bodyIK_y[fn] = ((total_X[fn] * sin_rot_Y[fn] * sin_rot_X[fn])
                  - (total_X[fn] * cos_rot_Y[fn] * cos_rot_X[fn] * sin_rot_Z[fn])
                  + (total_Z[fn] * cos_rot_Y[fn] * sin_rot_X[fn])
                  + (total_Z[fn] * cos_rot_X[fn] * sin_rot_Y[fn] * sin_rot_Z[fn])
                  + (total_Y[fn] * cos_rot_Z[fn] * cos_rot_X[fn])) - total_Y[fn];

  //==========LEG IK=========
  new_pos_x[fn] = feetPos_x[fn] + pos_x[fn] + bodyIK_x[fn];
  new_pos_y[fn] = feetPos_y[fn] - pos_y[fn] + bodyIK_y[fn];
  new_pos_z[fn] = feetPos_z[fn] + pos_z[fn] + bodyIK_z[fn];
  //print_New_pos(fn);
  cxFeetDistance[fn] = sqrt(new_pos_x[fn] * new_pos_x[fn] + new_pos_z[fn] * new_pos_z[fn]);

  IKWS[fn] = sqrt((cxFeetDistance[fn] - cx) * (cxFeetDistance[fn] - cx) + (new_pos_y[fn] * new_pos_y[fn]));
  IKA1[fn] = atan((cxFeetDistance[fn] - cx) / new_pos_y[fn]);
  IKA2[fn] = acos(((tb * tb) - (fm * fm) - (IKWS[fn] * IKWS[fn])) / (-2 * IKWS[fn] * fm));
  t_angle[fn] = acos(((IKWS[fn] * IKWS[fn]) - (tb * tb) - (fm * fm)) / (-2 * tb * fm));
  //print_IKWS(fn);
  GAMA[fn] = 90 - (atan(new_pos_z[fn] / new_pos_x[fn]) * float(180) / PI - cxRunAngle[fn]) - 90;
  ALPHA[fn] = (IKA1[fn] + IKA2[fn]) * float(180) / PI - 90;
  BETA[fn] = 90 - t_angle[fn] * float(180) / PI;

  if (fn <= 3) {
    GAMA[fn] = 90 + GAMA[fn] + calA[fn];
    ALPHA[fn] = 90 - ALPHA[fn] + calB[fn];
    BETA[fn] = 90 - BETA[fn] + calC[fn];
  } else {
    GAMA[fn] = 90 + GAMA[fn] + calA[fn];
    ALPHA[fn] = 90 + ALPHA[fn] + calB[fn];
    BETA[fn] = 90 + BETA[fn] + calC[fn];
  }
  servoAction(GAMA[fn], ALPHA[fn], BETA[fn], in_speed, fn);
}

void servoAction(float a, float b, float c, float ss, byte fn) {
  float s = ss / float(50);
  while ((posA[fn] != a) || (posB[fn] != b) || (posC[fn] != c)) {
    //=================1A
    if (posA[fn] < a) {
      if ((posA[fn] + s) <= a) posA[fn] = posA[fn] + s;
      else posA[fn] = a;
    }
    if (posA[fn] > a) {
      if ((posA[fn] - s) >= a) posA[fn] = posA[fn] - s;
      else posA[fn] = a;
    }
    //sCoxa[fn].write(posA[fn]);
    //======================1B
    if (posB[fn] < b) {
      if ((posB[fn] + s) <= b) posB[fn] = posB[fn] + s;
      else posB[fn] = b;
    }
    if (posB[fn] > b) {
      if ((posB[fn] - s) >= b) posB[fn] = posB[fn] - s;
      else posB[fn] = b;
    }
    //sFemur[fn].write(posB[fn]);
    //======================1C
    if (posC[fn] < c) {
      if ((posC[fn] + s) <= c) posC[fn] = posC[fn] + s;
      else posC[fn] = c;
    }
    if (posC[fn] > c) {
      if ((posC[fn] - s) >= c) posC[fn] = posC[fn] - s;
      else posC[fn] = c;
    }
    //sTibia[fn].write(posC[fn]);
  }
  sCoxa[fn].write(posA[fn]);
  sFemur[fn].write(posB[fn]);
  sTibia[fn].write(posC[fn]);
}

void gripper(float ar, float b, float ss) {
  float s = ss / 25;
  while ((posArm != ar) || (posGrip != b)) {
    //=================1A
    if (posArm < ar) {
      if (posArm + s <= ar) posArm = posArm + s;
      else posArm = ar;

    }
    if (posArm > ar) {
      if ((posArm - s) >= ar) posArm = posArm - s;
      else posArm = ar;
    }
    //======================1B
    if (posGrip < b) {
      if ((posGrip + s) <= b) posGrip = posGrip + s;
      else posGrip = b;
    }
    if (posGrip > b) {
      //Serial.println("GADANG ");
      if ((posGrip - s) >= b) posGrip = posGrip - s;
      else posGrip = b;
    }
    //======================1C
    /*
      if (posBase < c) {
      if ((posBase + s) <= c) posBase = posBase + s;
      else posBase = c;
      }
      if (posBase > c) {
      if ((posBase - s) >= c) posBase = posBase - s;
      else posBase = c;
      }
    */
    //Serial.print("posGrip ");
    //Serial.println(posGrip);
    sArm.write(posArm);
    sJapit.write(posGrip);
  }
}

void stopMovement() {
  for (int fn = 1; fn <= 6; fn++) {
    pos_x[fn] = in_px;
    pos_y[fn] = in_py;
    pos_z[fn] = in_pz;
    pos_a[fn] = in_pitch;
    pos_b[fn] = in_roll;
    pos_g[fn] = in_yaw;
    cal_IK(pos_x[fn], pos_y[fn],  pos_z[fn], pos_a[fn], pos_b[fn], pos_g[fn], fn);
  }
}
