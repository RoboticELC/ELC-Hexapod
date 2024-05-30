

#include <Servo.h>
#include <Math.h>
#include <Ramp.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

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

const double L_TF = 70.0; // panjang femur-tibia 70 mm
const double L_TO = 42.0; // panjang tibia-target 42 mm

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
        CartesianMoveLeg (JXActL, JYActL, JZActL, i);
      }
      else {
        CartesianMoveLeg (JXActR, JYActR, JZActR, i);
      }
    }
  
}

void loop() {
  // put your main code here, to run repeatedL_Oy:

  Belok_Kanan ();
  
//   if (Serial3.available () > 0) {
//    dataRead = Serial3.read ();
//    tampung = dataRead;
//   }
//
//   if (dataRead != tampung) {
//    tampung = dataRead;
//   }
//   else {
//    tampung = 'S';
//   }
//
//  if (tampung == 'F') {
//    Gerak_Maju ();
//  }
//  else if (tampung == 'R') {
//    Belok_Kanan ();
//  }
//  else if (tampung == 'L') {
//    Belok_Kiri ();
//  }
//  else if (tampung == 'B'){
//    Gerak_Mundur ();
//  }
//  else {
//    Serial.println ("Eror");
//    Berdiri ();
//  }
//  Serial.println ();

  //delay (3000);
   
   
//    int deg_tibia_kanan;
      
//    J1Act = J1Tar.update ();
//    J2Act = J2Tar.update ();
//    J3Act = J3Tar.update ();

//    
//    for (int deg = 0; deg <= 90; deg++) {
//      deg_tibia_kanan = map (deg, 0, 90, 180, 90);
//      
//      for (int i = 1; i < 7; i++) {
//        
//        if (i > 3) {
//          tibia[i].write (deg);
//        }
//        else {
//          tibia[i].write (deg_tibia_kanan);
//        }
//        
//      }
//      delay (50);
//    }
//    for (int deg = 90; deg >= 0; deg--) {
//      deg_tibia_kanan = map (deg, 0, 90, 180, 90);
//      
//      for (int i = 1; i < 7; i++) {
//        if (i > 3) {
//          tibia[i].write (deg);
//        }
//        else {
//          tibia[i].write (deg_tibia_kanan);
//        }
//      }
//      delay (50);
//    }
  
//  int deg_femur_6, deg_femur_kanan; 
//  
//  for (int deg = 90; deg <= 140; deg++) {
//    deg_femur_kanan = map (deg, 90, 140, 90, 45);
//    deg_tibia_kanan = map (deg, 0, 90, 180, 90);
//  
//    for (int i = 1; i < 7; i++) {
//      if (i > 3) {
//        femur[i].write (deg_femur_kanan);
//      }
//      else {
//        femur[i].write (deg);
//      }
//    }
//    
//    delay (50);
//  }
//  
//  for (int deg = 140; deg >= 90; deg--) {
//    deg_femur_kanan = map (deg, 90, 140, 90, 45);
//    deg_tibia_kanan = map (deg, 0, 90, 180, 90);
//    
//    for (int i = 1; i < 7; i++) {  
//      if (i > 3) {
//        femur[i].write (deg_femur_kanan);
//      }
//      else {
//        femur[i].write (deg);
//      }
//    }
//    
//    delay (50); 
//  }
  
}

void Belok_Kanan () {

     CartesianMoveLeg (0,9,65, 1);   //tes z,15 +- 1 cm  // Koordinat 1 (kaki 1-3) (0, 9, 65)
     CartesianMoveLeg (0,9,65, 3);  
     CartesianMoveLeg (0,-30,-24, 5);  //tes y,5 +- 1 cm batas 8 Koordinat 1 (kaki 4-6) (0, -30, -24)
    
      delay (100);
//    
      CartesianMoveLeg (20,9,65, 1);   
      CartesianMoveLeg (30,9,65, 3); 
      CartesianMoveLeg (10,-30,-24, 5);

      delay (100);
//    
      CartesianMoveLeg (20,0,0, 1);   
      CartesianMoveLeg (30,0,0, 3); 
      CartesianMoveLeg (10,0,0, 5);

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
      CartesianMoveLeg (10,-30,-24, 4);   
      CartesianMoveLeg (10,-30,-24, 6); 
      CartesianMoveLeg (30,9,65, 2);

      delay (100);
//    
      CartesianMoveLeg (10,0,0, 4);   
      CartesianMoveLeg (10,0,0, 6); 
      CartesianMoveLeg (30,0,0, 2);

      delay (700);

      CartesianMoveLeg (0,0,0, 4);   
      CartesianMoveLeg (0,0,0, 6); 
      CartesianMoveLeg (0,0,0, 2);

      delay (100);
  
}

void Belok_Kiri () {

     CartesianMoveLeg (0,9,65, 1);   //tes z,15 +- 1 cm  // Koordinat 1 (kaki 1-3) (0, 9, 65)
     CartesianMoveLeg (0,9,65, 3);  
     CartesianMoveLeg (0,-30,-24, 5);  //tes y,5 +- 1 cm batas 8 Koordinat 1 (kaki 4-6) (0, -30, -24)
    
      delay (100);
//    
      CartesianMoveLeg (-20,9,65, 1);   
      CartesianMoveLeg (-30,9,65, 3); 
      CartesianMoveLeg (-10,-30,-24, 5);

      delay (100);
//    
      CartesianMoveLeg (-20,0,0, 1);   
      CartesianMoveLeg (-30,0,0, 3); 
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
      CartesianMoveLeg (-30,9,65, 2);

      delay (100);
//    
      CartesianMoveLeg (-10,0,0, 4);   
      CartesianMoveLeg (-10,0,0, 6); 
      CartesianMoveLeg (-30,0,0, 2);

      delay (700);

      CartesianMoveLeg (0,0,0, 4);   
      CartesianMoveLeg (0,0,0, 6); 
      CartesianMoveLeg (0,0,0, 2);

      delay (100);
  
}

void Gerak_Maju () {
    
      CartesianMoveLeg (0,9,65, 1);   //tes z,15 +- 1 cm  // Koordinat 1 (kaki 1-3) (0, 9, 65)
      CartesianMoveLeg (0,9,65, 3);  
      CartesianMoveLeg (0,-30,-24, 5);  //tes y,5 +- 1 cm batas 8 Koordinat 1 (kaki 4-6) (0, -30, -24)
    
      delay (100);
//    
      CartesianMoveLeg (-20,9,65, 1);   
      CartesianMoveLeg (-30,9,65, 3); 
      CartesianMoveLeg (10,-30,-24, 5);

      delay (100);
//    
      CartesianMoveLeg (-20,0,0, 1);   
      CartesianMoveLeg (-30,0,0, 3); 
      CartesianMoveLeg (10,0,0, 5);

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
      CartesianMoveLeg (10,-30,-24, 4);   
      CartesianMoveLeg (10,-30,-24, 6); 
      CartesianMoveLeg (-30,9,65, 2);

      delay (100);
//    
      CartesianMoveLeg (10,0,0, 4);   
      CartesianMoveLeg (10,0,0, 6); 
      CartesianMoveLeg (-30,0,0, 2);

      delay (700);

      CartesianMoveLeg (0,0,0, 4);   
      CartesianMoveLeg (0,0,0, 6); 
      CartesianMoveLeg (0,0,0, 2);

      delay (100);
  
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
  Z += Z_Rest;

  double deg_cox = atan (X / Y) * (180 / PI);
  double H = sqrt ((Y * Y) + (X * X));
  double L_O = sqrt ((H * H) + (Z * Z));
  double deg_tibia = acos (   ((L_TF * L_TF) + (L_TO * L_TO) - (L_O * L_O))   /   (2 * L_TF * L_TO)   ) * (180 / PI);
  double B = acos (((L_O * L_O) + (L_TF * L_TF) - (L_TO * L_TO))   /   (2 * L_O * L_TF)   ) * (180 / PI);
  double A = atan (Z / H) * (180 / PI);  
  double deg_femur = (B + A);  
  
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
     femur[i].write (45 - deg2);
     tibia[i].write (deg3 + deg_TibiaLag - 90 ); 
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
