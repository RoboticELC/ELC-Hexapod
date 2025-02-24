
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Servo.h>
#include <Math.h>
#include <Ramp.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <VL53L0X.h>
#include <QMC5883LCompass.h>


#define nOP 5 //number of ping
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
float H1=1,H2=1,H3=1,H4=1,H5=1,H6=1;
float diff_X=1,diff_Y=1;
#define buzzerPin A1

//SoftwareSerial hc06(0,1);

Servo coxa[7];
Servo femur[7];
Servo tibia[7];


int tPin[] = {29, 25, 44, 43, 17};
int ePin[] = {27, 24, 42, 41, 16};
int dMax[] = {80, 80, 50, 50, 50};//jarak maksimum (mm)

NewPing pinG[nOP] = {NewPing(tPin[0], ePin[0], dMax[0]),
                     NewPing(tPin[1], ePin[1], dMax[1]),
                     NewPing(tPin[2], ePin[2], dMax[2]),
                     NewPing(tPin[3], ePin[3], dMax[3]),
                     NewPing(tPin[4], ePin[4], dMax[4]),
                    };
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

// Kaki kanan (ini di daerah +)
double JXActR = 0.0;  // -20 <= x <= 20  (titik 0 adalah 0 derajat dengan sumbu x) 0
double JYActR = 0.0;   // 0 <= y <= 8  (menjauh -> mendekat body) 0
double JZActR = 0.0;  // 0 <= z <= 15 (naik -> turun)  -7

// Kaki kiri (ini di daerah -)
double JXActL = 0.0;  // -20 <= x <= 20  (titik 0 adalah 0 derajat dengan sumbu x) 0
double JYActL = 0.0;   // -8 <= y <= 0  (mendekat -> menjauh body) 40
double JZActL = 0.0;  //  <= z <= 10 (naik -> turun) 70

rampDouble J1Tar = 0.0;
rampDouble J2Tar = -10.0;
rampDouble J3Tar = 40.0;

const double deg_TibiaLag = 15.4; 

char dataRead, tampung;
int diff_y=0,diff_lx,diff_rx,a,akeep,anew,anewkeep;
int stepbelok_x=30,stepbelok_z=40;
int Brake=0;
float OldCoordinate[3][7];
float CurrentCoordinate[3][7];

int currentState = 0;
void setup() {
  // put your setup code here, to run once:

  // batas atas femur = 140
  // batas bawah femur = 90
  // batas atas tibia = 90+30
  // batas bawah tibia = 90-30

  Serial3.begin (9600);
  Serial.begin (9600);

  for (int i = 0; i < 7; i++) {
    
    femur[i].attach (femurPin[i]);
    tibia[i].attach (tibiaPin[i]);
    coxa[i].attach (coxaPin[i]);
    
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

  // while (Serial3.available () < 0) {}
  Berdiri ();
  SetupGyro();

  // delay (1000);
  

  delay (1000);
    
}

void Berdiri () {

  // Berdiri
    for (int i = 1; i < 7; i++) {

      if (i > 3) {
        CartesianMoveLeg (JXActL, JYActL+diff_y, JZActL, i);
      }
      else {
        CartesianMoveLeg (JXActR, JYActR+diff_y, JZActR, i);
      }
    }
  
}

void Berdiri_Tangga () {

  //berdiri tangga
  CartesianMoveLeg (50,-20,-5, 1);   
  CartesianMoveLeg (20,diff_y,0, 3);   
  CartesianMoveLeg (-40,diff_y,0, 5);
  CartesianMoveLeg (-50,-20,-5, 4);  
  CartesianMoveLeg (-20,diff_y,0, 6);   
  CartesianMoveLeg (40,diff_y,0, 2);
  
}

int D_dpn,D_blkg,D_kiri,D_kanan;
void loop() {
   //getGyro();
  for (int i = 0; i < 7; i++) {
      if( i==2 || i==4 || i==6)
      {
        femur[i].attach (femurPin[i]);
        tibia[i].attach (tibiaPin[i]);
        coxa[i].attach (coxaPin[i]);
      }
    }
  if (Serial3.available () > 0) {
    
    dataRead = Serial3.read ();
    GetGyro();
    Serial.print("X="); Serial.println(AccX);
    // Serial.println(dataRead);
    // Serial.println("balik sini");
    if (dataRead == 'F') 
    {
      if(AccX<-0.35)
        Gerak_Tangga();
      else
        Gerak_Maju ();
    }


    else if (dataRead == 'B' ) 
      Gerak_Mundur ();

    else if (dataRead == 'R' ) 
      Belok_Kanan ();

    else if (dataRead == 'L' ) 
      Belok_Kiri ();

    else if (dataRead == 'Z' ) 
      for (int i = 0; i < 20; i++)
        Stabilizer();

    else 
    {
      if(AccX<-0.35)
        Berdiri_Tangga ();
      else
        Berdiri ();
    }

  }
  // put your main code here, to run repeatedL_Oy:

  // //Belok_Kanan ();
  // D_blkg=getDistancePing(0);//cm
  // D_dpn=getDistancePing(1);//cm
  //D_kiri=sensor_L.readRangeContinuousMillimeters()-30;//mm
  // D_kanan=sensor_R.readRangeContinuousMillimeters()-50;//mm
  
  // Serial.print("L= ");
  // Serial.println(D_kiri);
  // Serial.print(" |R= ");
  // Serial.print(D_kanan);
  // Serial.print(" |D= ");
  // Serial.print(D_dpn);
  // Serial.print(" |B= ");
  // Serial.println(D_blkg);

  // getDirection();
  // Serial.println(bearing);

  // if(D_kiri<700) 
  // {
  //   int interval=300;
  //   //------------------------------------// Pasangan 2 dorong belakang
  //     CartesianMoveLeg (-20,diff_y,-2, 6);
  //     CartesianMoveLeg (-50,-20,-5, 4);    
  //     CartesianMoveLeg (40,diff_y,0, 2);
  //     delay (interval);      
  //     //------------------------------------// Pasangan 2 angkat
  //     CartesianMoveLeg (-50,-20,40, 4);  
  //     CartesianMoveLeg (-30,10,55, 6);   
  //     CartesianMoveLeg (40,diff_y,40, 2);
  //     //delay (interval);  
  //   //------------------------------------// Pasangan 2 geser
  //     CartesianMoveLeg (-15,-20,40, 4);  
  //     CartesianMoveLeg (40,-20,55, 6);   
  //     CartesianMoveLeg (-5,diff_y,40, 2);
  //     delay (interval);         
  //   //------------------------------------// Pasangan 2 tancap
  //     CartesianMoveLeg (-15,-19,-10, 4);  
  //     CartesianMoveLeg (40,-20,25, 6);   
  //     CartesianMoveLeg (-5,diff_y,0, 2);
  //     delay (interval);
  //     //------------------------------------// Pasangan 1 angkat
  //     CartesianMoveLeg (50,-20,40, 1);   
  //     CartesianMoveLeg (20,diff_y,40, 3);   
  //     CartesianMoveLeg (-40,diff_y,40, 5);
  //   //------------------------------------// Pasangan 2 dorong belakang
  //     CartesianMoveLeg (40,-20,15, 6);
  //     CartesianMoveLeg (-50,-20,-5, 4);    
  //     CartesianMoveLeg (40,diff_y,0, 2);
  //     delay (interval/2);
  //     CartesianMoveLeg (30,-10,10, 6);
  //     delay (interval/4);
  //     CartesianMoveLeg (-20,diff_y,-2, 6); 
  //     delay (interval/4);


  // }


  // if(D_dpn>15)
  // {
  //   if(D_kanan<700) 
  //   {
  //     diff_lx=(-10);
  //     //beep();
  //   }
  //   else if(D_kiri<700) 
  //   {
  //     diff_rx=10;
  //     //beep();beep();
  //   }
  //   else 
  //   {
  //     diff_rx=0;
  //     diff_lx=0;
  //   }
  // }
  // else
  // {
  //   if(D_kanan>400) 
  //   {
  //     getDirection();
  //     new_bearing=bearing+3;
  //     if (new_bearing>16) new_bearing-=16;

  //     while(bearing!=new_bearing)
  //     {
  //       getDirection();
  //       Belok_Kanan();
  //     }
  //   }
  //   else if(D_kiri>400) 
  //   {
  //     getDirection();
  //     new_bearing=bearing-3;
  //     if (new_bearing<0) new_bearing+=16;

  //     while(bearing!=new_bearing)
  //     {
  //       getDirection();
  //       Belok_Kiri();
  //     }
  //   }
  // }



}

void Belok_Kanan () {
      int interval=10;
      int T=10;
      int tancap_z=0;
    //------------------------------------// Pasangan 1 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (0,diff_y,40,interval, 3); 
      CartesianMoveLeg_teratur (0,diff_y,40,interval, 5);  
      delay (T);
    }
    UpdateCoordinate(1);
//    
    //------------------------------------// Pasangan 2 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 4);  
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 2);
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,40,interval, 3);
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,40,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 1 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,tancap_z,interval, 1);    
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,tancap_z,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    Check_Remote('R');
    if(Brake==1){
      Brake=0;
      return;
    }
//
      //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0, diff_y, 40,interval, 4);   
      CartesianMoveLeg_teratur (0, diff_y, 40,interval, 6);   
      CartesianMoveLeg_teratur (0, diff_y, 40,interval, 2);  
      delay (T);
    }
    UpdateCoordinate(2);
//
      //------------------------------------// Pasangan 1 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 1);   
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 5);
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,40,interval, 4);    
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,40,interval, 6);   
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,40,interval, 2);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);

    
    
//    
    //------------------------------------// Pasangan 2 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,tancap_z,interval, 4);    
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (stepbelok_x,diff_y,tancap_z,interval, 2);
      delay (T);
    }
    UpdateCoordinate(2);
    Check_Remote('R');
    if(Brake==1){
      Brake=0;
      return;
    }
}

void Belok_Kiri () {
      int interval=10;
      int T=10;
      int tancap_z=0;
    //------------------------------------// Pasangan 1 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (0,diff_y,40,interval, 3); 
      CartesianMoveLeg_teratur (0,diff_y,40,interval, 5);  
      delay (T);
    }
    UpdateCoordinate(1);
//    
    //------------------------------------// Pasangan 2 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 4);  
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 2);
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,40,interval, 3);
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,40,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 1 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,tancap_z,interval, 1);    
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,tancap_z,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    Check_Remote('L');
    if(Brake==1){
      Brake=0;
      return;
    }
//
      //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0, diff_y, 40,interval, 4);   
      CartesianMoveLeg_teratur (0, diff_y, 40,interval, 6);   
      CartesianMoveLeg_teratur (0, diff_y, 40,interval, 2);  
      delay (T);
    }
    UpdateCoordinate(2);
//
      //------------------------------------// Pasangan 1 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 1);   
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (0,diff_y,tancap_z,interval, 5);
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,40,interval, 4);    
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,40,interval, 6);   
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,40,interval, 2);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 2 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,tancap_z,interval, 4);    
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (-stepbelok_x,diff_y,tancap_z,interval, 2);
      delay (T);
    }
    UpdateCoordinate(2);
    Check_Remote('L');
    if(Brake==1){
      Brake=0;
      return;
    }
}

void Gerak_Maju () {
      int interval=10;
      int T=10;
      int tancap_z=0;
    //------------------------------------// Pasangan 1 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 3); 
      CartesianMoveLeg_teratur (-20,diff_y,40,interval, 5);  
      delay (T);
    }
    UpdateCoordinate(1);
//    
    //------------------------------------// Pasangan 2 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20,diff_y,tancap_z,interval, 4);  
      CartesianMoveLeg_teratur (-20,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (20,diff_y,tancap_z,interval, 2);
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,40,interval, 3);
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,40,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 1 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,tancap_z,interval, 1);    
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,tancap_z,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    Check_Remote('F');
    if(Brake==1){
      Brake=0;
      return;
    }
//
      //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 4);   
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 6);   
      CartesianMoveLeg_teratur (20, diff_y, 40,interval, 2);  
      delay (T);
    }
    UpdateCoordinate(2);
//
      //------------------------------------// Pasangan 1 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,diff_y,tancap_z,interval, 1);   
      CartesianMoveLeg_teratur (20,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (-20,diff_y,tancap_z,interval, 5);
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,40,interval, 4);    
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,40,interval, 6);   
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,40,interval, 2);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 2 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,tancap_z,interval, 4);    
      CartesianMoveLeg_teratur (20+diff_lx,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (-15+diff_rx,diff_y,tancap_z,interval, 2);
      delay (T);
    }
    UpdateCoordinate(2);
    Check_Remote('F');
    if(Brake==1){
      Brake=0;
      return;
    }
}

void Gerak_Mundur () {
      int interval=10;
      int T=10;
      int tancap_z=0;
    //------------------------------------// Pasangan 1 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (-20,diff_y,40,interval, 3); 
      CartesianMoveLeg_teratur (20,diff_y,40,interval, 5);  
      delay (T);
    }
    UpdateCoordinate(1);
//    
    //------------------------------------// Pasangan 2 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20,diff_y,tancap_z,interval, 4);  
      CartesianMoveLeg_teratur (20,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (-20,diff_y,tancap_z,interval, 2);
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg_teratur (15+diff_rx,diff_y,40,interval, 1);  
      CartesianMoveLeg_teratur (15+diff_rx,diff_y,40,interval, 3);
      CartesianMoveLeg_teratur (-20+diff_lx,diff_y,40,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 1 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (15+diff_rx,diff_y,tancap_z,interval, 1);    
      CartesianMoveLeg_teratur (15+diff_rx,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (-20+diff_lx,diff_y,tancap_z,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    Check_Remote('B');
    if(Brake==1){
      Brake=0;
      return;
    }
//
      //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (20, diff_y, 40,interval, 4);   
      CartesianMoveLeg_teratur (20, diff_y, 40,interval, 6);   
      CartesianMoveLeg_teratur (-20, diff_y, 40,interval, 2);  
      delay (T);
    }
    UpdateCoordinate(2);
//
      //------------------------------------// Pasangan 1 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20,diff_y,tancap_z,interval, 1);   
      CartesianMoveLeg_teratur (-20,diff_y,tancap_z,interval, 3);   
      CartesianMoveLeg_teratur (20,diff_y,tancap_z,interval, 5);
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg_teratur (-20+diff_lx,diff_y,40,interval, 4);    
      CartesianMoveLeg_teratur (-20+diff_lx,diff_y,40,interval, 6);   
      CartesianMoveLeg_teratur (15+diff_rx,diff_y,40,interval, 2);
      delay (T);
    }
    UpdateCoordinate(1);
    UpdateCoordinate(2);
//    
    //------------------------------------// Pasangan 2 tancap
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur (-20+diff_lx,diff_y,tancap_z,interval, 4);    
      CartesianMoveLeg_teratur (-20+diff_lx,diff_y,tancap_z,interval, 6);   
      CartesianMoveLeg_teratur (15+diff_rx,diff_y,tancap_z,interval, 2);
      delay (T);
    }
    UpdateCoordinate(2);
    UpdateCoordinate(1);
    Check_Remote('B');
    if(Brake==1){
      Brake=0;
      return;
    }
}
void Gerak_Tangga()
{
      int interval=10;
      int T=10; 
//------------------------------------// Pasangan 1 angkat
      for(int i=0; i<interval;i++){
        CartesianMoveLeg_teratur (50,-20,40,interval, 1);   
        CartesianMoveLeg_teratur (20,diff_y,40,interval, 3);   
        CartesianMoveLeg_teratur (-40,diff_y,40,interval, 5);
        delay (T);
      }
      UpdateCoordinate(1);
//   
      
    //------------------------------------// Pasangan 2 dorong belakang
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur  (40,-20,15,interval, 6);
      CartesianMoveLeg_teratur  (-50,-20,-5,interval, 4);    
      CartesianMoveLeg_teratur  (40,diff_y,0,interval, 2);
      delay (T);
    }
    UpdateCoordinate(2);
//
    //------------------------------------// Pasangan 1 geser
    for(int i=0; i<interval;i++){
      CartesianMoveLeg_teratur  (5,-15,40,interval, 1);  
      CartesianMoveLeg_teratur  (-30,-20,40,interval, 3);
      CartesianMoveLeg_teratur  (1,diff_y,40,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
//    
    //------------------------------------// Pasangan 1 tancap
    for(int i=0; i<interval;i++){ 
      CartesianMoveLeg_teratur  (5,-15,-15,interval, 1);  
      CartesianMoveLeg_teratur  (-30,-20,10,interval, 3);
      CartesianMoveLeg_teratur  (10,diff_y,-2,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);
    Check_Remote('T');
    if(Brake==1){
      Brake=0;
      return;
    }

//
      //------------------------------------// Pasangan 2 angkat
    for(int i=0; i<interval;i++){ 
      CartesianMoveLeg_teratur  (-50,-20,40,interval, 4);  
      CartesianMoveLeg_teratur  (-30,10,55,interval, 6);   
      CartesianMoveLeg_teratur  (40,diff_y,40,interval, 2);
      delay (T);
    }
    UpdateCoordinate(2);
//
      //------------------------------------// Pasangan 1 dorong belakang
    for(int i=0; i<interval;i++){   
      CartesianMoveLeg_teratur  (20,diff_y,-2,interval, 3);  
      CartesianMoveLeg_teratur  (50,-20,-5,interval, 1);    
      CartesianMoveLeg_teratur  (-40,diff_y,0,interval, 5);
      delay (T);
    }
    UpdateCoordinate(1);

    //------------------------------------// Pasangan 2 geser
    for(int i=0; i<interval;i++){   
      CartesianMoveLeg_teratur  (-15,-20,40,interval, 4);  
      CartesianMoveLeg_teratur  (40,-20,55,interval, 6);   
      CartesianMoveLeg_teratur  (-5,diff_y,40,interval, 2);
      delay (T);
    } 
    UpdateCoordinate(2);

//    
    //------------------------------------// Pasangan 2 tancap
    for(int i=0; i<interval;i++){   
      CartesianMoveLeg_teratur  (-15,-19,-10,interval, 4);  
      CartesianMoveLeg_teratur  (40,-20,25,interval, 6);   
      CartesianMoveLeg_teratur  (-5,diff_y,0,interval, 2);
      delay (T);
    } 
    UpdateCoordinate(2);
    Check_Remote('T');
    if(Brake==1){
      Brake=0;
      return;
    }
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
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(100);
}

void Stabilizer() {

  GetGyro();
  // Serial.print("X="); Serial.print(AccX);
  // Serial.print("y=");Serial.print(AccY); 
  // Serial.print("   H1="); Serial.print(H1);Serial.print("  H2=");Serial.print(H2);Serial.print("  H3=");Serial.print(H3);Serial.print("  H4=");Serial.print(H4);Serial.print("   H5=");Serial.print(H5);Serial.print("  H6=");Serial.println(H6);

    H1=CurrentCoordinate[2][1];
    H3=CurrentCoordinate[2][3];
    H5=CurrentCoordinate[2][5];

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

    CartesianMoveLeg (OldCoordinate[0][1],0,H1, 1);    
    CartesianMoveLeg (OldCoordinate[0][3],0,H3, 3);   
    CartesianMoveLeg (OldCoordinate[0][5],0,H5, 5);
    OldCoordinate[2][1]=H1;
    OldCoordinate[2][3]=H3;
    OldCoordinate[2][5]=H5;
    CurrentCoordinate[2][1]=H1;
    CurrentCoordinate[2][3]=H3;
    CurrentCoordinate[2][5]=H5;

    for (int i = 0; i < 7; i++) {
      if( i==2 || i==4 || i==6)
      {
        femur[i].detach ();
        tibia[i].detach ();
        coxa[i].detach ();
      }
    }
  delay(5);
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

void Check_Remote(char x) {
    if (Serial3.available() > 0) {
        dataRead = Serial3.read();
        if (dataRead != x) {
            Brake=1;
        }
    }
}