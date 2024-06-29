#include <MPU6050.h>
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
QMC5883LCompass compass;
byte compasSt = 0;
int azimuth;
int fAzimuth;
byte bearing;
byte new_bearing;
float startHeadings;
float headingaSadow[361];
float relativeHeadings;
#define nOP 5 //number of ping
#define buzzerPin A1

//SoftwareSerial hc06(0,1);

Servo coxa[7];
Servo femur[7];
Servo tibia[7];

VL53L0X sensor_L;
VL53L0X sensor_R;

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
int diff_y=1,diff_lx,diff_rx,a,akeep,anew,anewkeep;
int stepbelok_x=10,stepbelok_z=40;

void setup() {
  // put your setup code here, to run once:

  // batas atas femur = 140
  // batas bawah femur = 90
  // batas atas tibia = 90+30
  // batas bawah tibia = 90-30

  Serial3.begin (9600);
  Serial.begin (9600);
  //hc06.begin(9600);
  setupCompass();
  setup_tof();

  for (int i = 0; i < 7; i++) {
    
    femur[i].attach (femurPin[i]);
    tibia[i].attach (tibiaPin[i]);
    coxa[i].attach (coxaPin[i]);
    
    if (i <= 3) { //kaki kiri
      femur[i].write (90);
      tibia[i].write (180);
    }
    else {
      femur[i].write (90+5);
      tibia[i].write (0);   
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
  //delay (50000000);


  Berdiri ();
  beep();
  delay (4000);
    
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

int D_dpn,D_blkg,D_kiri,D_kanan;
void loop() {
  // put your main code here, to run repeatedL_Oy:

  Gerak_Maju ();
  //Belok_Kanan ();
  D_blkg=getDistancePing(0);//cm
  D_dpn=getDistancePing(1);//cm
  D_kiri=sensor_L.readRangeContinuousMillimeters()-30;//mm
  D_kanan=sensor_R.readRangeContinuousMillimeters()-50;//mm
  
  // Serial.print("L= ");
  // Serial.print(D_kiri);
  // Serial.print(" |R= ");
  // Serial.print(D_kanan);
  // Serial.print(" |D= ");
  // Serial.print(D_dpn);
  // Serial.print(" |B= ");
  // Serial.println(D_blkg);

  getDirection();
  Serial.println(bearing);



  if(D_dpn>15)
  {
    if(D_kanan<70) 
    {
      diff_lx=(-10);
      //beep();
    }
    else if(D_kiri<70) 
    {
      diff_rx=10;
      //beep();beep();
    }
    else 
    {
      diff_rx=0;
      diff_lx=0;
    }
  }
  else
  {
    if(D_kanan>400) 
    {
      getDirection();
      new_bearing=bearing+3;
      if (new_bearing>16) new_bearing-=16;

      while(bearing!=new_bearing)
      {
        getDirection();
        Belok_Kanan();
      }
    }
    else if(D_kiri>400) 
    {
      getDirection();
      new_bearing=bearing-3;
      if (new_bearing<0) new_bearing+=16;

      while(bearing!=new_bearing)
      {
        getDirection();
        Belok_Kiri();
      }
    }
  }



}

void Belok_Kanan () {

     int interval=150;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (0,diff_y,stepbelok_z, 1);  
      CartesianMoveLeg (0,diff_y,stepbelok_z, 3); 
      CartesianMoveLeg (0,diff_y,stepbelok_z, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (0,diff_y,0, 4);  
      CartesianMoveLeg (0,diff_y,0, 6);   
      CartesianMoveLeg (0,diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (stepbelok_x,diff_y,stepbelok_z, 1);  
      CartesianMoveLeg (stepbelok_x,diff_y,stepbelok_z, 3);
      CartesianMoveLeg (stepbelok_x,diff_y,stepbelok_z, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (stepbelok_x,diff_y,0, 1);    
      CartesianMoveLeg (stepbelok_x,diff_y,0, 3);   
      CartesianMoveLeg (stepbelok_x,diff_y,0, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0,diff_y, stepbelok_z, 4);   
      CartesianMoveLeg (0,diff_y, stepbelok_z, 6);   
      CartesianMoveLeg (0,diff_y, stepbelok_z, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,diff_y,0, 1);   
      CartesianMoveLeg (0,diff_y,0, 3);   
      CartesianMoveLeg (0,diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (stepbelok_x,diff_y,stepbelok_z, 4);    
      CartesianMoveLeg (stepbelok_x,diff_y,stepbelok_z, 6);   
      CartesianMoveLeg (stepbelok_x,diff_y,stepbelok_z, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (stepbelok_x,diff_y,0, 4);    
      CartesianMoveLeg (stepbelok_x,diff_y,0, 6);   
      CartesianMoveLeg (stepbelok_x,diff_y,0, 2);
      delay (interval);
      
  
}

void Belok_Kiri () {

    int interval=150;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (0,diff_y,stepbelok_z, 1);  
      CartesianMoveLeg (0,diff_y,stepbelok_z, 3); 
      CartesianMoveLeg (0,diff_y,stepbelok_z, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (0,diff_y,0, 4);  
      CartesianMoveLeg (0,diff_y,0, 6);   
      CartesianMoveLeg (0,diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (-stepbelok_x,diff_y,stepbelok_z, 1);  
      CartesianMoveLeg (-stepbelok_x,diff_y,stepbelok_z, 3);
      CartesianMoveLeg (-stepbelok_x,diff_y,stepbelok_z, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (-stepbelok_x,diff_y,0, 1);    
      CartesianMoveLeg (-stepbelok_x,diff_y,0, 3);   
      CartesianMoveLeg (-stepbelok_x,diff_y,0, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0,diff_y, stepbelok_z, 4);   
      CartesianMoveLeg (0,diff_y, stepbelok_z, 6);   
      CartesianMoveLeg (0,diff_y, stepbelok_z, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,diff_y,0, 1);   
      CartesianMoveLeg (0,diff_y,0, 3);   
      CartesianMoveLeg (0,diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (-stepbelok_x,diff_y,stepbelok_z, 4);    
      CartesianMoveLeg (-stepbelok_x,diff_y,stepbelok_z, 6);   
      CartesianMoveLeg (-stepbelok_x,diff_y,stepbelok_z, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (-stepbelok_x,diff_y,0, 4);    
      CartesianMoveLeg (-stepbelok_x,diff_y,0, 6);   
      CartesianMoveLeg (-stepbelok_x,diff_y,0, 2);
      delay (interval);
    
}

void Gerak_Maju () {
      int interval=80;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (20,diff_y,40, 1);  
      CartesianMoveLeg (20,diff_y,40, 3); 
      CartesianMoveLeg (-20,diff_y,40, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (-20,diff_y,0, 4);  
      CartesianMoveLeg (-20,diff_y,0, 6);   
      CartesianMoveLeg (20,diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (-15+diff_rx,diff_y,40, 1);  
      CartesianMoveLeg (-15+diff_rx,diff_y,40, 3);
      CartesianMoveLeg (20+diff_lx,diff_y,40, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (-15+diff_rx,diff_y,0, 1);    
      CartesianMoveLeg (-15+diff_rx,diff_y,0, 3);   
      CartesianMoveLeg (20+diff_lx,diff_y,0, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (-20, diff_y, 40, 4);   
      CartesianMoveLeg (-20, diff_y, 40, 6);   
      CartesianMoveLeg (20, diff_y, 40, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (20,diff_y,0, 1);   
      CartesianMoveLeg (20,diff_y,0, 3);   
      CartesianMoveLeg (-20,diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (20+diff_lx,diff_y,40, 4);    
      CartesianMoveLeg (20+diff_lx,diff_y,40, 6);   
      CartesianMoveLeg (-15+diff_rx,diff_y,40, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (20+diff_lx,diff_y,0, 4);    
      CartesianMoveLeg (20+diff_lx,diff_y,0, 6);   
      CartesianMoveLeg (-15+diff_rx,diff_y,0, 2);
      delay (interval);
      
  
}

void Gerak_Mundur () {
    
      CartesianMoveLeg (0,9,65, 1);   //tes z,15 +- 1 cm  // Koordinat 1 (kaki 1-3) (0, 9, 65)
      CartesianMoveLeg (0,9,65, 3);  
      CartesianMoveLeg (0,-30,-24, 5);  //tes y,5 +- 1 cm batas 8 Koordinat 1 (kaki 4-6) (0, -30, -24)
    
      delay (100);
//    
      CartesianMoveLeg (20,9,65, 1);   
      CartesianMoveLeg (30,9,65, 3); 
      CartesianMoveLeg (-10,-30,-24, 5);

      delay (100);
//    
      CartesianMoveLeg (20,0,0, 1);   
      CartesianMoveLeg (30,0,0, 3); 
      CartesianMoveLeg (-10,0,0, 5);

      delay (700);

      CartesianMoveLeg (0,0,0, 1);   
      CartesianMoveLeg (0,0,0, 3); 
      CartesianMoveLeg (0,0,0, 5);
      
      delay (100);

      //------------------------------------// Pasangan 2
      
      CartesianMoveLeg (0,-30,-24, 4);   //tes z,15 +- 1 cm  // Koordinat 1 (kaki 1-3) (0, 9, 65)
      CartesianMoveLeg (0,-30,-24, 6);  
      CartesianMoveLeg (0, 9, 65, 2);  //tes y,5 +- 1 cm batas 8 Koordinat 1 (kaki 4-6) (0, -30, -24)
    
      delay (100);
//    
      CartesianMoveLeg (-10,-30,-24, 4);   
      CartesianMoveLeg (-10,-30,-24, 6); 
      CartesianMoveLeg (30,9,65, 2);

      delay (100);
//    
      CartesianMoveLeg (-10,0,0, 4);   
      CartesianMoveLeg (-10,0,0, 6); 
      CartesianMoveLeg (30,0,0, 2);

      delay (700);

      CartesianMoveLeg (0,0,0, 4);   
      CartesianMoveLeg (0,0,0, 6); 
      CartesianMoveLeg (0,0,0, 2);

      delay (100);
  
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
      if(femur_deg>140) femur_deg=140;
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
      else if (femur_deg<40) femur_deg=40;

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

float getDistancePing(byte n) {
  float distancePing = pinG[n].ping() / 57.0;
  if (distancePing == 0) distancePing = dMax[n];///10;
  return distancePing;
}


void setup_tof()
{
  pinMode(A13, OUTPUT); digitalWrite(A13, LOW);
  pinMode(26, OUTPUT); digitalWrite(26, LOW);
  Wire.begin();

  delay(100);
  digitalWrite(26, HIGH);
  delay(150);
  sensor_L.init(true);
  delay(100);
  sensor_L.setAddress((uint8_t)01);
  delay(100);

  digitalWrite(A13, HIGH);
  delay(150);
  sensor_R.init(true);
  delay(100);
  sensor_R.setAddress((uint8_t)02);
  delay(100);

  sensor_L.startContinuous();
  sensor_R.startContinuous();
  int tofSt = 1;
  Serial.println("setupTof ");
}

void setupCompass() {
  compass.init();
  compass.setCalibrationOffsets(723.00, -471.00, 469.00);
  compass.setCalibrationScales(0.79, 0.80, 2.04);

  //================
  compasSt = 1;
  //delay(100);
  startDirection();
  //delay(100);
  Serial.println("setupCompass ");
}

void getDirection() {
  //char direction[3];
  compass.read();
  azimuth = compass.getAzimuth();
  fAzimuth = compass.getAzimuth();
  bearing = compass.getBearing(azimuth);
  //compass.getDirection(direction, azimuth);
}
void testBacaArah(){
  getDirection();
  azimuth = compass.getAzimuth();
  Serial.print("azimuth "); Serial.println(azimuth);
  //Serial.print("azimuth "); Serial.println(azimuth);
}
void makeHeadingShadow() {
  getDirection();
  if (azimuth >= 0) {
    fAzimuth = azimuth;// - (22.5 * bearing);
  } else {
    fAzimuth = (azimuth + 360);// - (22.5 * bearing);
  }
}

void startDirection() {
  makeHeadingShadow();
  startHeadings = azimuth;
  int x = startHeadings;
  if (startHeadings != 180) {
    if (startHeadings < 180) {
      for (int i = 0; i <= 359; i++) {
        headingaSadow[i] = 180 - startHeadings + i; //arah pantat disimpan diarray?
        if (headingaSadow[i] > 359) headingaSadow[i] -= 360;
      }
    }
    if (startHeadings >= 180) {
      for (int i = 0; i <= 359; i++) {
        headingaSadow[i] = 180 - startHeadings + i;
        if (headingaSadow[i] < 0 ) headingaSadow[i] += 360;
      }
    }
  } else {
    for (int i = 0; i <= 359; i++) {
      headingaSadow[i] = i;
    }
  }
  Serial.print("startHeadings "); Serial.println(startHeadings); // Serial.print("/");
  Serial.print("bearing "); Serial.println(bearing); //Serial.print("/");
}

void getRelativeHeadings() {
  makeHeadingShadow();
  relativeHeadings = 180 - headingaSadow[fAzimuth];
}


void beep() {
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(100);
}