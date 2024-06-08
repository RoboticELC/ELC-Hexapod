
#include <Servo.h>
#include <Math.h>
#include <Ramp.h>
#include <EEPROM.h>

//SoftwareSerial hc06(0,1);

Servo coxa[7];
Servo femur[7];
Servo tibia[7];

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

const double Y_Rest =  70.0;
const double Z_Rest = -80.0;

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
int diff_y=1;

void setup() {
  // put your setup code here, to run once:

  // batas atas femur = 140
  // batas bawah femur = 90
  // batas atas tibia = 90+30
  // batas bawah tibia = 90-30

  Serial3.begin (9600);
  Serial.begin (9600);
  //hc06.begin(9600);
  
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

  Berdiri ();
  
  delay (5000);
  
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

void loop() {
  // put your main code here, to run repeatedL_Oy:

  if (Serial3.available () > 0) {
    
    dataRead = Serial3.read ();
    
    if (dataRead == 'F') {
      Gerak_Maju ();
    }
    else if (dataRead == 'B') {
      Gerak_Mundur ();
    }
    else if (dataRead == 'R') {
      Belok_Kanan ();
    }
    else if (dataRead == 'L') {
      Belok_Kiri ();
    }
    else {
      Berdiri ();
    }
    
  }

  
  
}

void Belok_Kiri () {

     int interval=150;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (0,9+diff_y,65, 1);  
      CartesianMoveLeg (0,9+diff_y,65, 3); 
      CartesianMoveLeg (0,9+diff_y,65, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 4);  
      CartesianMoveLeg (0,0+diff_y,0, 6);   
      CartesianMoveLeg (0,0+diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (-25,9+diff_y,65, 1);  
      CartesianMoveLeg (-25,9+diff_y,65, 3);
      CartesianMoveLeg (-25,9+diff_y,65, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (-25,0+diff_y,-1, 1);    
      CartesianMoveLeg (-25,0+diff_y,-1, 3);   
      CartesianMoveLeg (-25,0+diff_y,-1, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0, 9+diff_y, 65, 4);   
      CartesianMoveLeg (0, 9+diff_y, 65, 6);   
      CartesianMoveLeg (0, 9+diff_y, 65, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 1);   
      CartesianMoveLeg (0,0+diff_y,0, 3);   
      CartesianMoveLeg (0,0-diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (-25,9+diff_y,65, 4);    
      CartesianMoveLeg (-25,9+diff_y,65, 6);   
      CartesianMoveLeg (-25,9+diff_y,65, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (-25,0+diff_y,-1, 4);    
      CartesianMoveLeg (-25,0+diff_y,-1, 6);   
      CartesianMoveLeg (-25,0+diff_y,-1, 2);
      delay (interval);
  
}

void Gerak_Mundur () {

     int interval=150;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (0,9+diff_y,65, 1);  
      CartesianMoveLeg (0,9+diff_y,65, 3); 
      CartesianMoveLeg (0,9+diff_y,65, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 4);  
      CartesianMoveLeg (0,0+diff_y,0, 6);   
      CartesianMoveLeg (0,0+diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (25,9+diff_y,65, 1);  
      CartesianMoveLeg (25,9+diff_y,65, 3);
      CartesianMoveLeg (-25,9+diff_y,65, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (25,0+diff_y,-1, 1);    
      CartesianMoveLeg (25,0+diff_y,-1, 3);   
      CartesianMoveLeg (-25,0+diff_y,-1, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0, 9+diff_y, 65, 4);   
      CartesianMoveLeg (0, 9+diff_y, 65, 6);   
      CartesianMoveLeg (0, 9+diff_y, 65, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 1);   
      CartesianMoveLeg (0,0+diff_y,0, 3);   
      CartesianMoveLeg (0,0-diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (-25,9+diff_y,65, 4);    
      CartesianMoveLeg (-25,9+diff_y,65, 6);   
      CartesianMoveLeg (25,9+diff_y,65, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (-25,0+diff_y,-1, 4);    
      CartesianMoveLeg (-25,0+diff_y,-1, 6);   
      CartesianMoveLeg (25,0+diff_y,-1, 2);
      delay (interval);
  
}

void Gerak_Maju () {
  
      int interval=150;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (0,9+diff_y,65, 1);  
      CartesianMoveLeg (0,9+diff_y,65, 3); 
      CartesianMoveLeg (0,9+diff_y,65, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 4);  
      CartesianMoveLeg (0,0+diff_y,0, 6);   
      CartesianMoveLeg (0,0+diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (-25,9+diff_y,65, 1);  
      CartesianMoveLeg (-25,9+diff_y,65, 3);
      CartesianMoveLeg (25,9+diff_y,65, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (-25,0+diff_y,-1, 1);    
      CartesianMoveLeg (-25,0+diff_y,-1, 3);   
      CartesianMoveLeg (25,0+diff_y,-1, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0, 9+diff_y, 65, 4);   
      CartesianMoveLeg (0, 9+diff_y, 65, 6);   
      CartesianMoveLeg (0, 9+diff_y, 65, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 1);   
      CartesianMoveLeg (0,0+diff_y,0, 3);   
      CartesianMoveLeg (0,0-diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (25,9+diff_y,65, 4);    
      CartesianMoveLeg (25,9+diff_y,65, 6);   
      CartesianMoveLeg (-25,9+diff_y,65, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (25,0+diff_y,-1, 4);    
      CartesianMoveLeg (25,0+diff_y,-1, 6);   
      CartesianMoveLeg (-25,0+diff_y,-1, 2);
      delay (interval);
  
}

void Belok_Kanan () {
    
      int interval=150;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (0,9+diff_y,65, 1);  
      CartesianMoveLeg (0,9+diff_y,65, 3); 
      CartesianMoveLeg (0,9+diff_y,65, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 4);  
      CartesianMoveLeg (0,0+diff_y,0, 6);   
      CartesianMoveLeg (0,0+diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (25,9+diff_y,65, 1);  
      CartesianMoveLeg (25,9+diff_y,65, 3);
      CartesianMoveLeg (25,9+diff_y,65, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (25,0+diff_y,-1, 1);    
      CartesianMoveLeg (25,0+diff_y,-1, 3);   
      CartesianMoveLeg (25,0+diff_y,-1, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0, 9+diff_y, 65, 4);   
      CartesianMoveLeg (0, 9+diff_y, 65, 6);   
      CartesianMoveLeg (0, 9+diff_y, 65, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 1);   
      CartesianMoveLeg (0,0+diff_y,0, 3);   
      CartesianMoveLeg (0,0-diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (25,9+diff_y,65, 4);    
      CartesianMoveLeg (25,9+diff_y,65, 6);   
      CartesianMoveLeg (25,9+diff_y,65, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (25,0+diff_y,-1, 4);    
      CartesianMoveLeg (25,0+diff_y,-1, 6);   
      CartesianMoveLeg (25,0+diff_y,-1, 2);
      delay (interval);
  
}

void CartesianMoveLeg (double X, double Y, double Z, int i) {
  
  // OFFSET TO REST POSITION

  Y += Y_Rest;
  Z += Z_Rest;

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
  
   if (i > 3) {
     femur[i].write (map(90 - deg2,40,180,140,0));
     tibia[i].write (map(deg3 + deg_TibiaLag - 45,180,30,0,150 )); 
   }
   else {
     femur[i].write (90 - deg2);
     tibia[i].write (deg3 + deg_TibiaLag - 45); 
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
