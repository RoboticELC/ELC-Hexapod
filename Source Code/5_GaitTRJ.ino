float gaitPattern[120];
float trjPattern[120];

float trj_pattern(byte u) {
  float trjVal[in_nOS / 2];
  trjVal[u] = sin((180 / (q1 + 1) * u) / float(180) * PI);
  return trjVal[u];
}

float gait_pattern(byte sn, byte s) {
  float gp1[in_nOS];
  byte ngv;
  float gaitValue[in_nOS];

  if (s <= q1) gaitPattern[s] = (s - q3) * perStep / q3;
  if (s > q1) gaitPattern[s] = (-s + q4) * perStep / q3;

  if (s > 0) {
    gp1[sn] = s - sn + 1;
  }
  if (gp1[sn] > 0) {
    ngv = gp1[sn];
  } else {
    ngv = gp1[sn] + in_nOS;
  }
  gaitValue[s] = gaitPattern[ngv];
  return gaitValue[s];
}

void gaitCal(byte fn) {
  //1. Tripod_1, 2, Riplle
  byte ns11[] = {0, fn1_ns, fn6_ns, fn1_ns, fn6_ns, fn1_ns, fn6_ns};//gait 1, sm 2
  byte ns12[] = {0, fn1_ns, fn2_ns, fn3_ns, fn4_ns, fn5_ns, fn6_ns};//gait 1, sm 2

  if (in_gait == 1) { //1. Tripod_1
    sl[fn] = gait_pattern(ns11[fn], steep);
  }
  if (in_gait == 2) { //2. Ripple
    sl[fn] = gait_pattern(ns12[fn], steep);
  }
  //Serial.print("sl[fn] ");
  //Serial.println(sl[fn]);
}

void Ua7Cal(byte s, byte fn) {
  if (s == 1) {
    u[fn]++;
    Ua[fn] = trj_pattern(u[fn]) * in_lift;
  } else {
    Ua[fn] = 0;
    u[fn] = 0;
  }
}

void trajectoryCal(byte fn) {
  if (in_gait == 1) { //Tripod
    if ((fn == 1) || (fn == 3) || (fn == 5)) {
      if ((steep >= 1) and (steep <= q1))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }
    if ((fn == 2) || (fn == 4) || (fn == 6)) {
      if ((steep >= q2) and (steep <= in_nOS - 1))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }
  }

  //================================
  if (in_gait == 2) { //Ripple
    if (fn == 1) {
      if ((steep >= 1) and (steep <= 0.5 * in_nOS - 1))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }
    if (fn == 2) {
      //========================
      if ((steep >= 1) and (steep <= 0.25 * in_nOS - 2) or
          (steep >= 0.75 * in_nOS) and (steep <= in_nOS))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }
    if (fn == 3) {
      if ((steep >= 0.5 * in_nOS - 1) and (steep <= in_nOS - 3))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }

    if (fn == 4) {
      if ((steep >= 1) and (steep <= 0.5 * in_nOS - 3) or
          (steep >= in_nOS - 1) and (steep <= in_nOS))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }
    if (fn == 5) {
      if ((steep >= 0.25 * in_nOS) and (steep <= 0.75 * in_nOS - 2))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }
    if (fn == 6) {
      if ((steep >= 0.5 * in_nOS + 1) and (steep <= in_nOS - 1))
        Ua7Cal(1, fn); else Ua7Cal(0, fn);
    }
  }
}
