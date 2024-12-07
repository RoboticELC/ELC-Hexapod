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
const int button=34;
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

float OldCoordinate[3][7];
float CurrentCoordinate[3][7];

void setup() {
  Serial3.begin (9600);
  Serial.begin (9600);
  pinMode(button, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  //beep();

  int But=1;
  while(But==1)
  {
    But=digitalRead(button);
  }

  for (int i = 0; i < 7; i++) {
    
    // if( i!=1 && i!=3&& i!=5)
    // {
    // if( i!=2 && i!=4&& i!=6)
    // {
    femur[i].attach (femurPin[i]);
    tibia[i].attach (tibiaPin[i]);
    coxa[i].attach (coxaPin[i]);
    // }
    
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
  // CartesianMoveLeg (20+diff_lx,diff_y,-30, 5);
  delay (2000);
  SetupGyro();

  delay (3000);
  //delay(1000000);
    
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

void loop() {
  //Gerak();
  Gerak_Maju_Stabil ();
  //Gerak_Maju();
  //Stabilizer(2);
  //StabilJalan(1);
  // int But=digitalRead(button);
  // Serial.println( But);
}

void Tancap(int pasangan) { //1,3,5
  
  int za=40,zb=40,zc=40;
  int T=30;
  GetGyro();
  while(AccX<0.1 && AccX>-0.1)
  {
    za=za-2;
    zb=zb-2;
    if(pasangan==1)
    {
      CartesianMoveLeg (-15+diff_rx,diff_y,za, 1);
      CartesianMoveLeg (-15+diff_rx,diff_y,zb, 3);   
    }
    else if(pasangan==2)
    {
      CartesianMoveLeg (20+diff_lx,diff_y,za, 4);    
      CartesianMoveLeg (20+diff_lx,diff_y,zb, 6);   
    }
    
    if(za==-10 && zb==-10)
    {
      break;
      Serial.print("maxout ");
    }

    delay(T);
    GetGyro();
  }
    Serial.print("SIAP TURUN SEMUA ");
    Serial.print("X="); Serial.print(AccX);
    Serial.print(" y=");Serial.print(AccY); 
    Serial.print(" za="); Serial.print(za);
    Serial.print(" zb=");Serial.print(zb); 
  
  if(AccX<-0.05) //if pitch up
  {
    while(AccX<-0.05)
    {
      za=za-2;
        if(pasangan==1)
          CartesianMoveLeg (-15+diff_rx,diff_y,za, 1);
        else if(pasangan==2)
          CartesianMoveLeg (20+diff_lx,diff_y,za, 4);  

      if(za<-10)
        break;
            
      delay(T);
      GetGyro();
    }
    Serial.print("pitch up");
  }
  else if(AccX>0.05) //if pitch down
  {
    while(AccX>0.05)
    {
      zb=zb-2;
      if(pasangan==1)
        CartesianMoveLeg (-15+diff_rx,diff_y,zb, 3);   
      else if(pasangan==2)
        CartesianMoveLeg (20+diff_lx,diff_y,zb, 6);

      if(zb<-10)
        break;
     
      delay(T);
      GetGyro(); 
    }
    Serial.print("pitch down");
  }

  GetGyro(); 
  if(pasangan==1)
  {  
    while(AccY>-0.1)
    {
      zc=zc-2;
      CartesianMoveLeg (20+diff_lx,diff_y,zc, 5);

      if(zc<-10)
        break;

      delay(T);
      GetGyro();
    }
    while(AccY<-0.5)
    {
      zc=zc+2;
      CartesianMoveLeg (20+diff_lx,diff_y,zc, 5);
      delay(T);
      GetGyro();
    }

    // while(AccY<-0.05)
    // {
    //   zc=zc+2;
    //   CartesianMoveLeg (20+diff_lx,diff_y,zc, 5);
    //   delay(T); 
    //   GetGyro();
    // }

    OldCoordinate[2][1]=za+10;
    OldCoordinate[2][3]=zb+10;
    OldCoordinate[2][5]=zc+10;
    CurrentCoordinate[2][1]=za+10;
    CurrentCoordinate[2][3]=zb+10;
    CurrentCoordinate[2][5]=zc+10;

    // for(int j=1;j<=5;j=j+2)
    // { 
    //    CartesianMoveLeg (CurrentCoordinate[0][j],diff_y,CurrentCoordinate[2][j], j);
    // }
    Serial.print("Roll 1");

  }

  else if(pasangan==2)
  {
    while(AccY<0.1)
    {
      zc=zc-2;
      CartesianMoveLeg (-15+diff_rx,diff_y,zc, 2);

      if(zc<-10)
        break;

      delay(T);
      GetGyro();
    }
    while(AccY>0.05)
    {
      zc=zc+2;
      CartesianMoveLeg (-15+diff_rx,diff_y,zc, 2);
      delay(T);
      GetGyro();
    }
    // while(AccX>0.05)
    // {
    //   zc=zc+2;
    //   CartesianMoveLeg (20+diff_lx,diff_y,zc, 2);
    //   delay(T);
    //   GetGyro();
    // }
    OldCoordinate[2][4]=za+10;
    OldCoordinate[2][6]=zb+10;
    OldCoordinate[2][2]=zc+10;
    CurrentCoordinate[2][2]=za+10;
    CurrentCoordinate[2][4]=zb+10;
    CurrentCoordinate[2][6]=zc+10;

    // for(int j=2;j<=6;j=j+2)
    // { 
    //    CartesianMoveLeg (CurrentCoordinate[0][j],diff_y,CurrentCoordinate[2][j], j);
    // }
    Serial.print("Roll 2");
  }

  Serial.print("out tancap");
}


float H1=1,H2=1,H3=1,H4=1,H5=1,H6=1;
float diff_X=1,diff_Y=1;

void Stabilizer(int pasangan) {

  GetGyro();
  // Serial.print("X="); Serial.print(AccX);
  // Serial.print("y=");Serial.print(AccY); 
  Serial.print("   H1="); Serial.print(H1);Serial.print("  H2=");Serial.print(H2);Serial.print("  H3=");Serial.print(H3);Serial.print("  H4=");Serial.print(H4);Serial.print("   H5=");Serial.print(H5);Serial.print("  H6=");Serial.println(H6);

  if(pasangan==1)
  {
    H1=CurrentCoordinate[2][1];
    H3=CurrentCoordinate[2][3];
    H5=CurrentCoordinate[2][5];
  }
  else if(pasangan==2)
  {
    H2=CurrentCoordinate[2][2];
    H4=CurrentCoordinate[2][4];
    H6=CurrentCoordinate[2][6];
  }
  diff_X=abs(20*(AccX));
  diff_Y=abs(20*(AccY));

  if(AccX>0.05)
  {
    if(pasangan==1)
    {
      H1=H1+diff_X;
      H3=H3-diff_X;
    }
    else if (pasangan==2)
    {
      H4=H4+diff_X;
      H6=H6-diff_X;
    }
  }
  else if(AccX<-0.05)
  {
    if(pasangan==1)
    {
      H1=H1-diff_X;
      H3=H3+diff_X;
    }
    else if (pasangan==2)
    {
      H4=H4-diff_X;
      H6=H6+diff_X;
    }
  }

  if(AccY>0.05)
  {
    if(pasangan==1)
    {
      H5=H5-diff_Y;
      H1=H1+diff_Y;
      H3=H3+diff_Y;
    }
    else if (pasangan==2)
    {
      H2=H2+diff_Y;
      H4=H4-diff_Y;
      H6=H6-diff_Y;
    }
    
  }
  else if(AccY<-0.05)
  {
    if(pasangan==1)
    {
      H5=H5+diff_Y;
      H1=H1-diff_Y;
      H3=H3-diff_Y;
    }
    else if (pasangan==2)
    {
      H2=H2-diff_Y;
      H4=H4+diff_Y;
      H6=H6+diff_Y;
    }
  }

  if(H1<-28)
    H1=-28;
  if(H2<-28)
    H2=-28;
  if(H3<-25)
    H3=-25;
  if(H4<-28)
    H4=-28;
  if(H5<-28)
    H5=-28;
  if(H6<-25)
    H6=-25;

  if(pasangan==1)
  {
    CartesianMoveLeg (OldCoordinate[0][1],0,H1, 1);    
    CartesianMoveLeg (OldCoordinate[0][3],0,H3, 3);   
    CartesianMoveLeg (OldCoordinate[0][5],0,H5, 5);
    OldCoordinate[2][1]=H1;
    OldCoordinate[2][3]=H3;
    OldCoordinate[2][5]=H5;
    CurrentCoordinate[2][1]=H1;
    CurrentCoordinate[2][3]=H3;
    CurrentCoordinate[2][5]=H5;
  }
  else if(pasangan==2)
  {
    CartesianMoveLeg (OldCoordinate[0][2],0,H2, 2);    
    CartesianMoveLeg (OldCoordinate[0][4],0,H4, 4);   
    CartesianMoveLeg (OldCoordinate[0][6],0,H6, 6);
    OldCoordinate[2][2]=H2;
    OldCoordinate[2][4]=H4;
    OldCoordinate[2][6]=H6;
    CurrentCoordinate[2][2]=H2;
    CurrentCoordinate[2][4]=H4;
    CurrentCoordinate[2][6]=H6;
  }
  delay(5);
}

void CartesianMoveLeg_teratur(double X_baru, double Y_baru, double Z_baru, 
int interval, int i){

    CurrentCoordinate[0][i]+=(X_baru - OldCoordinate[0][i])/interval;
    CurrentCoordinate[1][i]+=(Y_baru - OldCoordinate[1][i])/interval;
    CurrentCoordinate[2][i]+=(Z_baru - OldCoordinate[2][i])/interval;

    CartesianMoveLeg (CurrentCoordinate[0][i],CurrentCoordinate[1][i],CurrentCoordinate[2][i],i);
}

void UpdateCoordinate(int pasangan)
{
  for(int i=0;i<3;i++)
  {
    if(pasangan==1)
    for(int j=1;j<=5;j=j+2)
    { 
       OldCoordinate[i][j]=CurrentCoordinate[i][j];
    }
    if(pasangan==2)
    for(int j=2;j<=6;j=j+2)
    { 
       OldCoordinate[i][j]=CurrentCoordinate[i][j];
    }
  }
}

void CartesianMoveLeg (double X, double Y, double Z, int i) {
  
  // OFFSET TO REST POSITION

  Y += Y_Rest;
  if(i==2 || i==5)
  Z += Z_Rest-10;
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
  Serial.print(AccX);
  Serial.print(" ");
  Serial.println(AccY);
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

void PersBidang(int Z1,int Z3,int Z5) {
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

void Stabiling(int pasangan)
{
  AccX=0.06;
  while(AccX>0.05 || AccX<-0.05 || AccY>0.05 || AccY<-0.05)
    Stabilizer(pasangan);
}
void Gerak_Maju_Stabil () {
    int interval=20;
    int T=40, delaying=100;
    //------------------------------------// Pasangan 1 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 3); 
      CartesianMoveLeg_teratur (-20,diff_y,40,interval, 5); 

      if(i==15) 
      for(int j=2;j<=6;j=j+2)
      { 
        CartesianMoveLeg (CurrentCoordinate[0][j],diff_y,CurrentCoordinate[2][j], j);
      }
      // if(i>15) 
      //   Stabilizer(2);
      delay (T);
    }
    UpdateCoordinate(1);
    Serial.println("1 angkat");
delay(delaying);

    Stabiling(2);
    Serial.println("STABIL SIAP");
//    
delay(delaying);
    //------------------------------------// Pasangan 2 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20,diff_y,OldCoordinate[2][4]+10,interval, 4);  
      CartesianMoveLeg_teratur (-20,diff_y,OldCoordinate[2][6],interval, 6);   
      CartesianMoveLeg_teratur (20,diff_y,OldCoordinate[2][2],interval, 2);

    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg_teratur (-15,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (-15,diff_y,41,interval, 3);
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 5);
      //Stabilizer(2);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
    Serial.println("2 dorong"); 
delay(delaying);
//  
    //------------------------------------// Pasangan 1 tancap
    Stabiling(2);
    Serial.println("STABIL SIAP");
delay(delaying);

    Tancap(1);
    Serial.println("1 tancap");
delay(delaying);
//
    //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 4);   
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 6);   
      CartesianMoveLeg_teratur (20, diff_y, 40,interval, 2);  
      if(i==15) 
        for(int j=1;j<=5;j=j+2)
        { 
          CartesianMoveLeg (CurrentCoordinate[0][j],diff_y,CurrentCoordinate[2][j], j);
        }
        
      // if(i>15)
      //   Stabilizer(1);
      delay (T);
    }
    UpdateCoordinate(2);
    Serial.println("2 angkat");
delay(delaying);


//    
    // Serial.println("x");
    // Serial.print(OldCoordinate[0][1]);
    // Serial.print(OldCoordinate[0][3]);
    // Serial.println(OldCoordinate[0][5]);
    // Serial.println("y");
    // Serial.print(OldCoordinate[1][1]);
    // Serial.print(OldCoordinate[1][3]);
    // Serial.println(OldCoordinate[1][5]);
    // Serial.println("z");
    // Serial.print(OldCoordinate[2][1]);
    // Serial.print(OldCoordinate[2][3]);
    // Serial.println(OldCoordinate[2][5]);
    Stabiling(1);
    Serial.println("STABIL SIAP");
delay(delaying);
    //------------------------------------// Pasangan 1 dorong belakang (masalah)
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,0,OldCoordinate[2][1],interval, 1);   
      CartesianMoveLeg_teratur (20,0,OldCoordinate[2][3],interval, 3);   
      CartesianMoveLeg_teratur (-20,0,OldCoordinate[2][5],interval, 5);
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 4);    
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 6);   
      CartesianMoveLeg_teratur (-15,diff_y,40,interval, 2);
      //Stabilizer(1);
      delay (T);
      Serial.print("1");
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
    Serial.println("1 dorong");
delay(delaying);

    Stabiling(1);
    Serial.println("STABIL SIAP");
delay(delaying);
//    
    //------------------------------------// Pasangan 2 tancap
//    
      Tancap(2);
      Serial.println("2 tancap");
delay(delaying);
//    
}





void Gerak_Maju () {
      int interval=200;
    //------------------------------------// Pasangan 1 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 3); 
      CartesianMoveLeg_teratur (-20,diff_y,40,interval, 5);  
      delay (1);
    }
    UpdateCoordinate(1);
//    
    //------------------------------------// Pasangan 2 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20,diff_y,-10,interval, 4);  
      CartesianMoveLeg_teratur (-20,diff_y,-25,interval, 6);   
      CartesianMoveLeg_teratur (20,diff_y,-15,interval, 2);
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,45,interval, 3);
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,40,interval, 5);
      delay (1);
    }
    UpdateCoordinate(2);
    UpdateCoordinate(1);
//    
    //------------------------------------// Pasangan 1 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,-10,interval, 1);    
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,-10,interval, 3);   
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,-10,interval, 5);
      delay (1);
    }
    UpdateCoordinate(1);
//
      //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 4);   
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 6);   
      CartesianMoveLeg_teratur (20, diff_y, 40,interval, 2);  
      delay (1);
    }
    UpdateCoordinate(2);
//
      //------------------------------------// Pasangan 1 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,diff_y,-10,interval, 1);   
      CartesianMoveLeg_teratur (20,diff_y,-25,interval, 3);   
      CartesianMoveLeg_teratur (-20,diff_y,-15,interval, 5);
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,40,interval, 4);    
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,45,interval, 6);   
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,40,interval, 2);
      delay (1);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 2 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,-10,interval, 4);    
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,-10,interval, 6);   
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,-10,interval, 2);
      delay (1);
    }
    UpdateCoordinate(2);
}

void Gerak () {
    OldCoordinate[2][1]=-10;
    OldCoordinate[2][3]=-10;
    OldCoordinate[2][5]=-12;
    int interval=20;
    //------------------------------------// Pasangan 1 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-15,0,OldCoordinate[2][1],interval, 1);    
      CartesianMoveLeg_teratur (-15,0,OldCoordinate[2][3],interval, 3);   
      CartesianMoveLeg_teratur (20,0,OldCoordinate[2][5],interval, 5);
      delay (50);
    }
    delay(500);
    UpdateCoordinate(1);

          //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 4);   
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 6);   
      CartesianMoveLeg_teratur (20, diff_y, 40,interval, 2);  
      delay (50);
    }
    UpdateCoordinate(2);
          //------------------------------------// Pasangan 1 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,0,OldCoordinate[2][1],interval, 1);   
      CartesianMoveLeg_teratur (20,0,OldCoordinate[2][3],interval, 3);   
      CartesianMoveLeg_teratur (-20,0,OldCoordinate[2][5],interval, 5);
      //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,40,interval, 4);    
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,45,interval, 6);   
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,40,interval, 2);
      delay (50);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//
}