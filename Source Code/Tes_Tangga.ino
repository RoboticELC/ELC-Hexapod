
#include <QMC5883LCompass.h>

#include <Servo.h>
#include <Math.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>

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

const double deg_TibiaLag = 15.4; 

char dataRead, tampung;
int diff_y=1,a,akeep,anew,anewkeep;
int stepbelok_x=10,stepbelok_z=20;
QMC5883LCompass compass;

void setup() {
  // put your setup code here, to run once:

  // batas atas femur = 140
  // batas bawah femur = 90
  // batas atas tibia = 90+30
  // batas bawah tibia = 90-30

  Serial3.begin (9600);
  Serial.begin (9600);
  //hc06.begin(9600);
  compass.init();
  hitungcompass();
  akeep=a;

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
  // Berdiri ();
  // delay(7000);
  CartesianMoveLeg (-10,-10,0, 3); //kaki depan dorong
  CartesianMoveLeg (80,-10,30, 1); //kaki belakang dorong
  CartesianMoveLeg (-10,0,0, 5); // kaki tengah dorong
  CartesianMoveLeg (10,-10,0, 6);//kaki depan dorong 
  CartesianMoveLeg (-80,-10,30, 4); //kaki belakang dorong
  CartesianMoveLeg (10,0,0, 2); //kaki tengah dorong
  delay(5000);

  // CartesianMoveLeg (-20,20,30, 4); //kaki belakang pijak
  // CartesianMoveLeg (20,20,30, 1); //kaki belakang pijak
  //   delay(5000000);

  //CartesianMoveLeg (-40,-10,0, 6); //kaki belakang pijak
  
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

  int interval = 400;
  int pindah =30;

  //pasanagan 1
  CartesianMoveLeg (-10,-10,30, 3); //kaki depan angkat
  CartesianMoveLeg (80,-10,60, 1); //kaki belakang angkat
  CartesianMoveLeg (0,10,30, 5); // kaki tengah angkat
  CartesianMoveLeg (10,-10,0, 6); //kaki depan dorong
  CartesianMoveLeg (-80,-10,30, 4); //kaki belakang dorong
  CartesianMoveLeg (10,0,0, 2); // kaki tengah dorong
  delay(interval);

  CartesianMoveLeg (-45,-40,30, 3); //kaki depan angkat maju
  CartesianMoveLeg (20,-20,40, 1); //kaki belakang angkat maju
  CartesianMoveLeg (30,10,30, 5); // kaki tengah angkat maju
  delay(interval);

  CartesianMoveLeg (-55,-40,0, 3); //kaki depan pijak
  CartesianMoveLeg (20,20,30, 1); //kaki belakang pijak
  CartesianMoveLeg (30,10,20, 5); // kaki tengah pijak
  delay(interval);

  //pasangan 2
  CartesianMoveLeg (10,-10,30, 6); //kaki depan angkat
  CartesianMoveLeg (-80,-10,60, 4); //kaki belakang angkat
  CartesianMoveLeg (0,10,30, 2); // kaki tengah angkat
  CartesianMoveLeg (-10,-10,0, 3); //kaki depan dorong
  CartesianMoveLeg (80,-10,30, 1); //kaki belakang dorong
  CartesianMoveLeg (-10,0,0, 5); // kaki tengah dorong
  delay(interval);
//////////////////////////////////////////

  CartesianMoveLeg (10,-10,30, 6); //kaki depan angkat
  CartesianMoveLeg (-80,-10,40, 4); //kaki belakang angkat
  CartesianMoveLeg (0,10,30, 2); // kaki tengah angkat
  delay(interval);

  CartesianMoveLeg (45,-40,30, 6); //kaki depan angkat maju
  CartesianMoveLeg (-20,-20,60, 4); //kaki belakang angkat maju
  CartesianMoveLeg (-30,10,30, 2); // kaki tengah angkat maju
  delay(interval);

  CartesianMoveLeg (55,-30,15, 6); //kaki depan pijak
  CartesianMoveLeg (-20,20,30, 4); //kaki belakang pijak
  CartesianMoveLeg (-30,10,20, 2); // kaki tengah pijak
  delay(interval);







  //pasangan 1
  // CartesianMoveLeg (-45,-40,40, 3); //kaki depan angkat
  // delay(interval);
  // CartesianMoveLeg (-95,-30,50, 3); //kaki depan angkat maju
  // delay(interval);
  // CartesianMoveLeg (-85,-30,20, 3); //kaki depan pijak
  // delay(interval);
  // CartesianMoveLeg (-70,-15,50, 4);//kaki belakang angkat
  // delay(interval);
  // CartesianMoveLeg (-25,-5,50, 4);//kaki belakang angkat maju
  // delay(interval);
  // CartesianMoveLeg (-25,-5,10 , 4); //kaki belakang pijak
  // delay(interval);
  // delay(1000000);

  //pasangan 2
  // CartesianMoveLeg (40,-40,50, 6); // kaki depan angkat
  // delay(interval);
  // tibia[6].write (0);
  // delay(200);
  // CartesianMoveLeg (100,-30,50, 6); // kaki depan angkat maju
  // delay(interval);
  // CartesianMoveLeg (80,-30,15, 6); //kaki depan pijak
  // delay(interval);
  // CartesianMoveLeg (30,0,40, 1); //kaki belakang angkat
  // delay(interval);
  // CartesianMoveLeg (-40,0,40, 1); //kaki belakang angkat maju
  // delay(interval);
  // CartesianMoveLeg (-40,0,10, 1); //kaki belakang pijak
  // delay(interval);

  //pasangan 3
  // CartesianMoveLeg (0,0,40, 2);//kaki tengah angkat 
  // CartesianMoveLeg (0,0,40, 5);//kaki tengah angkt 
  // delay(interval);

  // CartesianMoveLeg (-55,0,40, 2);//kaki tengah angkat maju
  // CartesianMoveLeg (55,0,40, 5);//kaki tengah angkt maju
  // delay(interval);

  // CartesianMoveLeg (-65,-20,15, 2); //kaki tengah pijak
  // CartesianMoveLeg (65,-20,10, 5); // kaki tengah pijak
  // delay(1000000);


  //joget2
  /*
  CartesianMoveLeg (-30,-30+pindah,0, 3); //kaki depan pijak
  CartesianMoveLeg (0,-30-pindah,0, 4); //kaki belakang pijak
  CartesianMoveLeg (0,-30+pindah,0, 1); //kaki belakang pijak
  CartesianMoveLeg (30,-30-pindah,0, 6); //kaki depan pijak
  CartesianMoveLeg (-30,-30+pindah,0, 2); //kaki tengah tancap
  CartesianMoveLeg (30,-30-pindah,0, 5); // kaki tengah tancap
  delay(interval);

  CartesianMoveLeg (-30,-30-pindah,0, 3); //kaki depan pijak
  CartesianMoveLeg (0,-30+pindah,0, 4); //kaki belakang pijak
  CartesianMoveLeg (0,-30-pindah,0, 1); //kaki belakang pijak
  CartesianMoveLeg (30,-30+pindah,0, 6); //kaki depan pijak
  CartesianMoveLeg (-30,-30-pindah,0, 2); //kaki tengah tancap
  CartesianMoveLeg (30,-30+pindah,0, 5); // kaki tengah tancap
  delay(interval);*/
  
  //tes koordinat
 /*
  CartesianMoveLeg (-60,-30,30, 3); //kaki depan pijak
  CartesianMoveLeg (0,-10,50, 4); //kaki belakang pijak
  CartesianMoveLeg (0,-10,50, 1); //kaki belakang pijak
  CartesianMoveLeg (60,-30,30, 6); //kaki depan pijak
  CartesianMoveLeg (-30,0,40, 2); //kaki tengah pijak
  CartesianMoveLeg (30,0,40, 5); // kaki tengah pijak
  delay(2000);
  CartesianMoveLeg (-30,-40,0, 3); //kaki depan dorong
  CartesianMoveLeg (30,-40,0, 6);//kaki depan dorong 
  CartesianMoveLeg (0,10,10, 2); //kaki tengah dorong
  CartesianMoveLeg (0,10,10, 5); // kaki tengah dorong
  CartesianMoveLeg (30,0,0, 1); //kaki belakang dorong
  CartesianMoveLeg (-30,0,0 , 4); //kaki belakang dorong
  delay(5000);*/

  // CartesianMoveLeg (30,0,0, 2); //kaki tengah dorong
  // CartesianMoveLeg (-30,0,0, 5); // kaki tengah dorong
  // CartesianMoveLeg (-30,-40,0, 3); //kaki depan dorong
  // CartesianMoveLeg (30,-40,0, 6);//kaki depan dorong 
  // CartesianMoveLeg (60,-30,-5, 1); //kaki belakang dorong
  // CartesianMoveLeg (-60,-30,-5 , 4); //kaki belakang dorong
  // delay(2000);

  //jalan maju tangga
  /*
  CartesianMoveLeg (0,0,40, 3); //kaki depan angkat
  delay(100);
  CartesianMoveLeg (-60,-30,40, 4);//kaki belakang angkat
  delay(interval);

  CartesianMoveLeg (-60,-30,30, 3); //kaki depan pijak
  delay(100);
  CartesianMoveLeg (0,-10,50, 4); //kaki belakang pijak
  delay(interval);

  CartesianMoveLeg (60,-30,40, 1); //kaki belakang angkat
  delay(100);
  CartesianMoveLeg (0,0,40, 6); // kaki depan angkat
  delay(interval);

  CartesianMoveLeg (0,-10,50, 1); //kaki belakang pijak
  delay(100);
  CartesianMoveLeg (60,-30,30, 6); //kaki depan pijak
  delay(interval);

  CartesianMoveLeg (0,0,40, 2);//kaki tengah angkat triangle
  delay(100);
  CartesianMoveLeg (0,0,40, 5);//kaki tengah angkt triangle
  delay(interval);

  CartesianMoveLeg (-30,0,40, 2); //kaki tengah pijak
  delay(100);
  CartesianMoveLeg (30,0,40, 5); // kaki tengah pijak
  delay(interval);

  CartesianMoveLeg (-30,-40,0, 3); //kaki depan dorong
  CartesianMoveLeg (30,-40,0, 6);//kaki depan dorong 
  CartesianMoveLeg (0,10,10, 2); //kaki tengah dorong
  CartesianMoveLeg (0,10,10, 5); // kaki tengah dorong
  CartesianMoveLeg (30,0,0, 1); //kaki belakang dorong
  CartesianMoveLeg (-30,0,0 , 4); //kaki belakang dorong
  delay(interval);*/


  /*
    CartesianMoveLeg (30,0,0, 2); //kaki tengah dorong
    CartesianMoveLeg (-30,0,0, 5); // kaki tengah dorong
    delay(3000);


    CartesianMoveLeg (30,0,40, 2); //kaki tengah angkat
    CartesianMoveLeg (-30,0,40, 5); // kaki tengah angkat
    delay(3000);

    CartesianMoveLeg (0,0,40, 5);//kaki tengah angkat triangle
    CartesianMoveLeg (0,0,40, 5);//kaki tengah angkt triangle

    CartesianMoveLeg (-30,0,40, 2); //kaki tengah angkat maju
    CartesianMoveLeg (30,0,40, 5); // kaki tengah angkat maju
    delay(3000);

    CartesianMoveLeg (-30,0,0, 2); //kaki tengah tancap
    CartesianMoveLeg (30,0,0, 5); // kaki tengah tancap
    delay(3000);
  */


  
  // put your main code here, to run repeatedL_Oy:
  /*
  CartesianMoveLeg (0,0,50, 3); //kaki depan angkat
  //CartesianMoveLeg (0,0,50, 6); // kaki depan angkat
  delay(3000);

  CartesianMoveLeg (-60,-30,40, 3); //kaki depan pijak
  //CartesianMoveLeg (60,-30,40, 6); //kaki depan pijak
  delay(3000);

  CartesianMoveLeg (0,0,0, 3); //kaki depan dorong
  //CartesianMoveLeg (0,0,0, 6);//kaki depan dorong 
  delay(3000);
*/




  /* KAKI
  CartesianMoveLeg (60,-30,-5, 1); //kaki belakang dorong
  CartesianMoveLeg (-60,-30,-5 , 4); //kaki belakang dorong
  delay(3000);

  
  CartesianMoveLeg (60,-30,40, 1); //kaki belakang angkat
  //CartesianMoveLeg (-60,-30,40, 4);//kaki belakang angkat
  delay(3000);

   CartesianMoveLeg (0,0,50, 1); //kaki belakang pijak
  //CartesianMoveLeg (0,0,50, 4); //kaki belakang pijak
  delay(3000);

  //CartesianMoveLeg (-40,7,60, 3);
  //CartesianMoveLeg (40,7,60, 6);
  */
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
      CartesianMoveLeg (stepbelok_x,9+diff_y,stepbelok_z, 1);  
      CartesianMoveLeg (stepbelok_x,9+diff_y,stepbelok_z, 3);
      CartesianMoveLeg (stepbelok_x,9+diff_y,stepbelok_z, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (stepbelok_x,0+diff_y,-1, 1);    
      CartesianMoveLeg (stepbelok_x,0+diff_y,-1, 3);   
      CartesianMoveLeg (stepbelok_x,0+diff_y,-1, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0, 9+diff_y, stepbelok_z, 4);   
      CartesianMoveLeg (0, 9+diff_y, stepbelok_z, 6);   
      CartesianMoveLeg (0, 9+diff_y, stepbelok_z, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 1);   
      CartesianMoveLeg (0,0+diff_y,0, 3);   
      CartesianMoveLeg (0,0-diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (stepbelok_x,9+diff_y,stepbelok_z, 4);    
      CartesianMoveLeg (stepbelok_x,9+diff_y,stepbelok_z, 6);   
      CartesianMoveLeg (stepbelok_x,9+diff_y,stepbelok_z, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (stepbelok_x,0+diff_y,-1, 4);    
      CartesianMoveLeg (stepbelok_x,0+diff_y,-1, 6);   
      CartesianMoveLeg (stepbelok_x,0+diff_y,-1, 2);
      delay (interval);
      
  
}

void Belok_Kiri () {

     int interval=150;
    //------------------------------------// Pasangan 1 angkat
      CartesianMoveLeg (0,9+diff_y,stepbelok_z, 1);  
      CartesianMoveLeg (0,9+diff_y,stepbelok_z, 3); 
      CartesianMoveLeg (0,9+diff_y,stepbelok_z, 5);  
      delay (interval);
//    
    //------------------------------------// Pasangan 2 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 4);  
      CartesianMoveLeg (0,0+diff_y,0, 6);   
      CartesianMoveLeg (0,0+diff_y,0, 2);
//
    //------------------------------------// Pasangan 1 geser
      CartesianMoveLeg (-stepbelok_x,9+diff_y,stepbelok_z, 1);  
      CartesianMoveLeg (-stepbelok_x,9+diff_y,stepbelok_z, 3);
      CartesianMoveLeg (-stepbelok_x,9+diff_y,stepbelok_z, 5);
      delay (interval);
//    
    //------------------------------------// Pasangan 1 tancap
      CartesianMoveLeg (-stepbelok_x,0+diff_y,-1, 1);    
      CartesianMoveLeg (-stepbelok_x,0+diff_y,-1, 3);   
      CartesianMoveLeg (-stepbelok_x,0+diff_y,-1, 5);
      delay (interval);
//
      //------------------------------------// Pasangan 2 angkat
      CartesianMoveLeg (0, 9+diff_y, stepbelok_z, 4);   
      CartesianMoveLeg (0, 9+diff_y, stepbelok_z, 6);   
      CartesianMoveLeg (0, 9+diff_y, stepbelok_z, 2);  
      delay (interval);
//
      //------------------------------------// Pasangan 1 dorong belakang
      CartesianMoveLeg (0,0+diff_y,0, 1);   
      CartesianMoveLeg (0,0+diff_y,0, 3);   
      CartesianMoveLeg (0,0-diff_y,0, 5);
//    
    //------------------------------------// Pasangan 2 geser
      CartesianMoveLeg (-stepbelok_x,9+diff_y,stepbelok_z, 4);    
      CartesianMoveLeg (-stepbelok_x,9+diff_y,stepbelok_z, 6);   
      CartesianMoveLeg (-stepbelok_x,9+diff_y,stepbelok_z, 2);
      delay (interval);
//    
    //------------------------------------// Pasangan 2 tancap
      CartesianMoveLeg (-stepbelok_x,0+diff_y,-1, 4);    
      CartesianMoveLeg (-stepbelok_x,0+diff_y,-1, 6);   
      CartesianMoveLeg (-stepbelok_x,0+diff_y,-1, 2);
      delay (interval);
      
  
}

void Gerak_Maju () {
      int interval=200;
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

void hitungcompass() {
  // Read compass values
  compass.read();

  // Return Azimuth reading
  a = compass.getAzimuth();
  if(a<0)
  {
    a=360+a;
  }

 Serial.print("A: ");
  Serial.print(a);
  Serial.println();
}
