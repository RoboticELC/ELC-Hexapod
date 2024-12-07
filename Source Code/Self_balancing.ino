#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Servo.h>
#include <Math.h>
#include <Ramp.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include "Wire.h"

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
#define buzzerPin A1

//SoftwareSerial hc06(0,1);

Servo coxa[7];
Servo femur[7];
Servo tibia[7];

int tPin[] = {29, 25, 44, 43, 17};
int ePin[] = {27, 24, 42, 41, 16};
int dMax[] = {80, 80, 50, 50, 50};//jarak maksimum (mm)

//double deg_fem1, deg_fem2, 

const int coxaPin[] = {0, 45, 38, 35, 28, 18, 6};
const int femurPin[] = {0, 46, 39, 33, 31, 19, 8}; //Femur kaki 4 ada di pin 8 (femur[6]), femur[0] itu leher
const int tibiaPin[] = {0, 47, 40, 37, 30, 22, 9}; //Tibia kaki 4 ada di pin 9 (tibia[6]), tibia[0] itu gripper

/* Bentuk kartesian kaki kiri
                ^ +z
                |
                |
                |
                |
  -y <- - - - - -
  
 */

 /* Bentuk kartesian kaki kanan
       ^ +z
       |
       |
       |
       |
       - - - - - > +y
 */

const double L_TO = 70.0; // panjang femur-tibia 70 mm
const double L_TF = 42.0; // panjang tibia-target 42 mm

const double Y_Rest =  60.0;
const double Z_Rest = -60.0;

rampDouble J1Tar = 0.0;
rampDouble J2Tar = -10.0;
rampDouble J3Tar = 40.0;

const double deg_TibiaLag = 15.4; 

char dataRead, tampung;
int diff_y=0,diff_lx,diff_rx,a,akeep,anew,anewkeep;
int stepbelok_x=10,stepbelok_z=40;

double OldCoordinate[3][7];
double CurrentCoordinate[3][7];

void setup() {
  Serial3.begin (9600);
  Serial.begin (9600);

  for (int i = 0; i < 7; i++) {
    
    if( i!=4 && i!=6&& i!=2)
    {
    femur[i].attach (femurPin[i]);
    tibia[i].attach (tibiaPin[i]);
    coxa[i].attach (coxaPin[i]);
    }
    
    if (i <= 3) { //kaki kiri
      femur[i].write (90);
      tibia[i].write (180);

      // femur[i].write (50);
      // tibia[i].write (30);
    }
    else {
      femur[i].write (90+5);
      tibia[i].write (0);

      // femur[i].write (140);
      // tibia[i].write (150); 
    }
      femur[0].write (20);
      tibia[0].write (150);

  }

  coxa[1].write (90+45);
  coxa[2].write (90);
  coxa[3].write (90-45);
  coxa[4].write (90-45);
  coxa[5].write (90);
  coxa[6].write (90+45);  
  //delay (500000);
  Berdiri ();

  SetupGyro();
  delay (5000);

  pinMode(buzzerPin, OUTPUT);
  beep();
 
    
}

void Berdiri () {

    for (int i = 1; i < 7; i++) {
        CartesianMoveLeg (0, 0, 0, i);
    }

    for(int i=0;i<3;i++)
    { 
      for(int j=1;j<7;j++)
        OldCoordinate[i][j]=0;
    }
  
}
float H1=1,H2=1,H3=1,H4=1,H5=1,H6=1;
float diff_X=1,diff_Y=1;
void loop() {
  GetGyro();
  Serial.print("X="); Serial.print(AccX);
  Serial.print("y=");Serial.print(AccY); Serial.print("   H1="); Serial.print(H1);Serial.print("  H2=");Serial.print(H2);Serial.print("  H3=");Serial.print(H3);Serial.print("  H4=");Serial.print(H4);Serial.print("   H5=");Serial.print(H5);Serial.print("  H6=");Serial.println(H6);

  diff_X=abs(20*(AccX));
  diff_Y=abs(20*(AccY));

  if(AccX>0.05)
  {
    H1=H1+diff_X;
    H3=H3-diff_X;
  }
  else if(AccX<-0.05)
  {
    H1=H1-diff_X;
    H3=H3+diff_X;
  }

  if(AccY>0.05)
  {
    H5=H5-diff_Y;
    H1=H1+diff_Y;
    H3=H3+diff_Y;
    
  }
  else if(AccY<-0.05)
  {
    H5=H5+diff_Y;
    H1=H1-diff_Y;
    H3=H3-diff_Y;
  }

  if(H1<-28)
    H1=-28;
  if(H3<-25)
    H3=-25;
  if(H5<-28)
    H5=-28;

  PersGaris(H1,H3,H5);
  H2=(H1+H3)/2;
  if(H5<0)
  {
    //H2 = H2+abs(H2*0.1);
    H4 = H4+abs(H4*0.7);
    H6 = H6+abs(H6*0.6);
  }
  else 
  {
    //H2 = H2-abs(H2*0.5);
    H4 = H4-abs(H4*0.5);
    H6 = H6-abs(H6*0.5);
  }
  SetHeight(H1,H2,H3,H4,H5,H6);

    delay(3);
}

void SetHeight(int a,int b, int c, int d,int e, int f){
    CartesianMoveLeg (0,0,a, 1);    
    CartesianMoveLeg (0,0,b, 2);   
    CartesianMoveLeg (0,0,c, 3);
    CartesianMoveLeg (0,0,d, 4);    
    CartesianMoveLeg (0,0,e, 5);   
    CartesianMoveLeg (0,0,f, 6);
}

void CartesianMoveLeg_teratur(double X_baru, double Y_baru, double Z_baru, 
int interval, int i){

    CurrentCoordinate[0][i]+=(X_baru - OldCoordinate[0][i])/interval;
    CurrentCoordinate[1][i]+=(Y_baru - OldCoordinate[1][i])/interval;
    CurrentCoordinate[2][i]+=(Z_baru - OldCoordinate[2][i])/interval;

    CartesianMoveLeg (CurrentCoordinate[0][i],CurrentCoordinate[1][i],CurrentCoordinate[2][i],i);
}

void UpdateCoordinate()
{
  for(int i=0;i<3;i++)
  {
    for(int j=1;j<7;j++)
       OldCoordinate[i][j]=CurrentCoordinate[i][j];
  }
}

void CartesianMoveLeg (double X, double Y, double Z, int i) {
  
  // OFFSET TO REST POSITION

  Y += Y_Rest;
  if(i==2 || i==5)
  Z += Z_Rest-8;
  else 
  Z += Z_Rest;

  if(i==3)
  X-=40;
  else if(i==6)
  X+=40;
  else if(i==1)
  X+=30;
  else if(i==4)
  X-=30;

  double deg_cox = atan (X / Y) * (180 / PI);
  double H = sqrt ((Y * Y) + (X * X));
  double L_O = sqrt ((H * H) + (Z * Z));
  double deg_tibia = acos (   ((L_TF * L_TF) + (L_TO * L_TO) - (L_O * L_O))   /   (2 * L_TF * L_TO)   ) * (180 / PI);
  double B = acos (((L_O * L_O) + (L_TF * L_TF) - (L_TO * L_TO))   /   (2 * L_O * L_TF)   ) * (180 / PI);
  double A = atan (Z / H) * (180 / PI);  
  double deg_femur = (B + A);  //harusnya (B + (90-A))??, patokan mulai sudutnya -z???
  
  UpdateCartesian (deg_cox, deg_femur, deg_tibia, i);
//  Serial.print ("cox: ");
//  Serial.print (deg_cox);
//  Serial.print (" | femur: ");
//  Serial.print (deg_femur);
//  Serial.print (" | tibia: ");
//  Serial.println (deg_tibia);
//  delay (100);

}

void UpdateCartesian (double deg1, double deg2, double deg3, int i) {
  
  // MOVE TO POSITION
  double femur_deg,tibia_deg,coxa_deg;
    if (i > 3) {
      femur_deg = map(90 - deg2,40,180,140,0);
      if(femur_deg>130) femur_deg=130;
      else if (femur_deg<0) femur_deg=0;

      tibia_deg = map(deg3 + deg_TibiaLag - 45,180,30,0,150 );
      if(tibia_deg>150) tibia_deg=150;
      else if (tibia_deg<0) tibia_deg=0;

      femur[i].write (femur_deg);
      tibia[i].write (tibia_deg); 
    }
    else {
      femur_deg = 90 - deg2;
      if(femur_deg>180) femur_deg=180;
      else if (femur_deg<50) femur_deg=50;

      tibia_deg = deg3 + deg_TibiaLag - 45;
      if(tibia_deg>180) tibia_deg=180;
      else if (tibia_deg<30) tibia_deg=30;

      femur[i].write (femur_deg);
      tibia[i].write (tibia_deg); 
    }

   if (i == 1 || i == 6) {
     coxa[i].write (90+45 - deg1);
   }
   if (i == 2 || i == 5) {
     coxa[i].write (90 - deg1);
   }
   if (i == 3 || i == 4) {
     coxa[i].write (90-45 - deg1);
   }
  
//  coxa[1].write (90+45 - deg1);
//  coxa[2].write (90 - deg1);
//  coxa[3].write (90-45 - deg1);
//
//  coxa[4].write (90-45 - deg1);
//  coxa[5].write (90 - deg1);
//  coxa[6].write (90+45 - deg1);
  
  //delay (100);
  
}

void beep() {

  analogWrite(buzzerPin, 255);
  delay(100);
  analogWrite(buzzerPin, 0);
  delay(100);
}

void GetGyro(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void SetupGyro() {
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void PersGaris(int Z1,int Z3,int Z5) {
  int V13X= 21;
  int V13Y= 0;
  int V13Z= Z3-Z1;
  int V35X= -10;
  int V35Y= 18;
  int V35Z= Z5-Z3;

  int Cross_i= (V13Y*V35Z)-(V13Z*V35Y);
  int Cross_j= (V13X*V35Z)-(V13Z*V35X);
  int Cross_k= (V13X*V35Y)-(V13Y*V35X);

  //H2 = ((Cross_k*Z1)-(Cross_i*(15-26))-(Cross_j*(3-5)))/Cross_k;
  H4 = -((Cross_k*Z1)-(Cross_i*(26-5))-(Cross_j*(21-5)))/Cross_k;
  H6 = -((Cross_k*Z1)-(Cross_i*(5-5))-(Cross_j*(21-5)))/Cross_k;

}