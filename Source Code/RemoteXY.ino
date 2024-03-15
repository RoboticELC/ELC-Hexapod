#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_SERIAL Serial3
#define REMOTEXY_SERIAL_SPEED 9600


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 555 bytes
{ 255, 16, 0, 121, 0, 36, 2, 16, 60, 4, 5, 19, 2, 13, 33, 33, 1, 2, 26, 47,
  129, 0, 1, 2, 37, 3, 1, 107, 72, 101, 120, 97, 112, 111, 100, 32, 82, 111, 98, 111,
  116, 32, 67, 111, 110, 116, 114, 111, 108, 108, 101, 114, 0, 131, 5, 56, 57, 21, 5, 1,
  2, 31, 77, 97, 105, 110, 32, 80, 97, 103, 101, 0, 131, 4, 78, 57, 21, 5, 2, 2,
  31, 67, 97, 108, 105, 98, 114, 97, 116, 105, 111, 110, 0, 4, 160, 18, 27, 84, 5, 2,
  2, 94, 4, 160, 18, 18, 84, 5, 2, 2, 94, 4, 160, 18, 10, 84, 5, 2, 2, 94,
  4, 0, 59, 26, 5, 24, 1, 2, 94, 4, 0, 73, 26, 5, 24, 1, 2, 94, 4, 32,
  66, 26, 5, 24, 1, 2, 94, 129, 0, 59, 52, 6, 2, 1, 94, 83, 112, 101, 101, 100,
  0, 129, 0, 74, 52, 2, 2, 1, 94, 76, 105, 102, 116, 0, 129, 0, 68, 52, 2, 2,
  1, 94, 89, 0, 4, 176, 40, 11, 52, 5, 1, 2, 94, 129, 0, 61, 17, 8, 2, 1,
  94, 68, 105, 114, 101, 99, 116, 105, 111, 110, 0, 129, 0, 0, 20, 9, 3, 2, 17, 70,
  101, 109, 117, 114, 0, 129, 0, 0, 11, 9, 3, 2, 17, 84, 105, 98, 105, 97, 0, 129,
  0, 0, 29, 7, 3, 2, 17, 67, 111, 120, 97, 0, 4, 48, 88, 23, 5, 19, 1, 2,
  94, 4, 176, 80, 46, 21, 5, 1, 2, 94, 129, 0, 89, 41, 3, 2, 1, 94, 82, 111,
  108, 108, 0, 129, 0, 88, 52, 4, 2, 1, 94, 80, 105, 116, 99, 104, 0, 4, 0, 40,
  26, 5, 24, 1, 2, 94, 4, 32, 47, 26, 5, 24, 1, 2, 94, 129, 0, 40, 52, 4,
  2, 1, 94, 65, 114, 109, 0, 129, 0, 47, 52, 4, 2, 1, 94, 71, 114, 105, 112, 0,
  129, 0, 17, 60, 5, 2, 1, 94, 71, 97, 109, 97, 0, 67, 1, 39, 22, 7, 3, 1,
  2, 26, 11, 67, 1, 46, 22, 7, 3, 1, 2, 26, 11, 67, 1, 58, 22, 7, 3, 1,
  2, 26, 11, 67, 1, 65, 22, 7, 3, 1, 2, 26, 11, 67, 1, 72, 22, 7, 3, 1,
  2, 26, 11, 67, 1, 87, 20, 7, 3, 1, 2, 26, 11, 67, 1, 62, 7, 7, 3, 1,
  2, 26, 11, 3, 132, 11, 54, 17, 5, 1, 2, 26, 67, 1, 17, 50, 7, 3, 1, 2,
  26, 11, 3, 134, 0, 42, 53, 10, 2, 2, 26, 129, 0, 3, 54, 2, 3, 2, 17, 49,
  0, 67, 4, 11, 11, 8, 3, 2, 2, 26, 11, 67, 4, 11, 20, 8, 3, 2, 2, 26,
  11, 67, 4, 11, 29, 8, 3, 2, 2, 26, 11, 129, 0, 12, 54, 2, 3, 2, 17, 50,
  0, 129, 0, 21, 54, 2, 3, 2, 17, 51, 0, 129, 0, 30, 54, 2, 3, 2, 17, 52,
  0, 129, 0, 39, 54, 2, 3, 2, 17, 53, 0, 129, 0, 48, 54, 2, 3, 2, 17, 54,
  0, 1, 1, 77, 43, 23, 7, 2, 2, 31, 83, 65, 86, 69, 0
};

// this structure defines all the variables and events of your control interface
struct {

  // input variables
  int8_t joystick_1_x; // from -100 to 100
  int8_t joystick_1_y; // from -100 to 100
  int8_t slider_c1; // =-100..100 slider position
  int8_t slider_f1; // =-100..100 slider position
  int8_t slider_t1; // =-100..100 slider position
  int8_t slider_Speed; // =0..100 slider position
  int8_t slider_Lift; // =0..100 slider position
  int8_t slider_Y; // =-100..100 slider position
  int8_t slider_Direction; // =-100..100 slider position
  int8_t slider_Roll; // =-100..100 slider position
  int8_t slider_Pitch; // =-100..100 slider position
  int8_t slider_Arm; // =0..100 slider position
  int8_t slider_Grip; // =-100..100 slider position
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ...
  uint8_t select_2; // =0 if select position A, =1 if position B, =2 if position C, ...
  uint8_t button_save; // =1 if button pressed, else =0

  // output variables
  char text_Arm[11];
  char text_Grip[11];
  char text_Speed[11];
  char text_Y[11];
  char text_Lift[11];
  char text_Roll[11];
  char text_Dir[11];
  char text_Gama[11];
  char text_t[11];
  char text_f[11];
  char text_c[11];

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)


void setupXY() {
  RemoteXY_Init ();
  RemoteXY.slider_c1 = map(calA[1], -15, 15, -100, 100);
  RemoteXY.slider_f1 = map(calB[1], -15, 15, -100, 100);
  RemoteXY.slider_t1 = map(calC[1], -15, 15, -100, 100);
  RemoteXY.slider_Lift = 30;
  RemoteXY.slider_Speed = 20;
  Serial.println("setupXY ");
}

int legCal = 1, oldLegCal = 0;
float newLift, newYaw, newSpeed;
float oldLift, oldYaw, oldSpeed, oldPitch, oldRoll, oldPz, oldPx, oldPy, oldArm, oldGrip;
void loopXY() {
  RemoteXY_Handler ();
  XYst = RemoteXY.connect_flag;
  if (XYst == 1) {
    actionXY();
    calibXY();
  }
}

void actionXY() {
  if (oldLift != RemoteXY.slider_Lift) {
    oldLift = RemoteXY.slider_Lift;
    in_lift = mapFloat(RemoteXY.slider_Lift, 0, 100, 0, 20);
  }
  if (oldYaw != RemoteXY.slider_Direction) {
    oldYaw = RemoteXY.slider_Direction;
    in_yaw = mapFloat(RemoteXY.slider_Direction, -100, 100, 15, -15);
  }
  if (oldSpeed != RemoteXY.slider_Speed) {
    oldSpeed = RemoteXY.slider_Speed;
    in_speed = mapFloat(RemoteXY.slider_Speed, 0, 100, 0, 10);
  }
  if (oldPitch != RemoteXY.slider_Pitch) {
    oldPitch = RemoteXY.slider_Pitch;
    in_pitch = mapFloat(RemoteXY.slider_Pitch, -100, 100, -5, 5);
  }
  if (oldRoll != RemoteXY.slider_Roll) {
    oldRoll = RemoteXY.slider_Roll;
    in_roll = mapFloat(RemoteXY.slider_Roll, -100, 100, -5, 5);
  }
  if (oldPz != RemoteXY.joystick_1_y) {
    oldPz = RemoteXY.joystick_1_y;
    in_pz = mapFloat(RemoteXY.joystick_1_y, -100, 100, 20, -20);
  }
  if (oldPx != RemoteXY.joystick_1_x) {
    oldPx = RemoteXY.joystick_1_x;
    in_px = mapFloat(RemoteXY.joystick_1_x, -100, 100, -20, 20);
  }
  if (oldPy != RemoteXY.slider_Y) {
    oldPy = RemoteXY.slider_Y;
    in_py = mapFloat(RemoteXY.slider_Y, -100, 100, 15, -15);
  }
  if (oldArm != RemoteXY.slider_Arm) {
    oldArm = RemoteXY.slider_Arm;
    posArm = mapFloat(RemoteXY.slider_Arm, 0, 100, 40, 180);
    gripper(posArm, posGrip, 2);
  }
  if (oldGrip != RemoteXY.slider_Grip) {
    oldGrip = RemoteXY.slider_Grip;
    posGrip = mapFloat(RemoteXY.slider_Grip, 0, 100, 20, 180);
    gripper(posArm, posGrip, 2);
  }
}

void calibXY() {
  for (int n = 0; n <= 5; n++) {
    if (RemoteXY.select_2 == n) {
      legCal = n + 1;
      if (legCal != oldLegCal) {
        RemoteXY.slider_c1 = mapFloat(calA[legCal], -15, 15, -100, 100);
        RemoteXY.slider_f1 = mapFloat(calB[legCal], -15, 15, -100, 100);
        RemoteXY.slider_t1 = mapFloat(calC[legCal], -15, 15, -100, 100);
        dtostrf(calA[legCal], 0, 1, RemoteXY.text_c);
        dtostrf(calB[legCal], 0, 1, RemoteXY.text_f);
        dtostrf(calC[legCal], 0, 1, RemoteXY.text_t);

        Serial.print("legCal ");
        Serial.println(legCal);
        beep();
        oldLegCal = legCal;

        //calA[legCal] = mapFloat(RemoteXY.slider_c1, -100, 100, -15, 15);
        //calB[legCal] = mapFloat(RemoteXY.slider_f1, -100, 100, -15, 15);
        //calC[legCal] = mapFloat(RemoteXY.slider_t1, -100, 100, -15, 15);
      }
    }
  }
  if (RemoteXY.button_save != 0) {
    EEPROM.writeFloat(4 * legCal + 347, calA[legCal]);
    EEPROM.writeFloat(4 * legCal + 371, calB[legCal]);
    EEPROM.writeFloat(4 * legCal + 395, calC[legCal]);
    beep();
  }
  calA[legCal] = mapFloat(RemoteXY.slider_c1, -100, 100, -15, 15);
  calB[legCal] = mapFloat(RemoteXY.slider_f1, -100, 100, -15, 15);
  calC[legCal] = mapFloat(RemoteXY.slider_t1, -100, 100, -15, 15);
}
