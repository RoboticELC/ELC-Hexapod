float sum_error;
float last_error = 0;
long t01;
long eplaseTime;

double yawPid(float ref, float in, float Kp, float Ki, float Kd ) {
  //eplaseTime = (millis() - t01); // waktu 1 siklus program
  double Op;
  eplaseTime = 1;
  double error = ref - in;
  double deltaError = error - last_error;
  sum_error += last_error;
  double pTerm = Kp * error;
  double iTerm = Ki * constrain(sum_error, -100, 100) * eplaseTime;
  double dTerm = ((Kd / eplaseTime) * deltaError);
  last_error = error;
  double gain = constrain((pTerm + iTerm + dTerm), -100, 100);
  Op =  mapFloat(gain, -100, 100, 10, -10);
  return Op;
  t01 = millis();
}
