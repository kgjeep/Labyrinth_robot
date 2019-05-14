





#include <NewPing.h>

#define TRIGGER_PINL  A0// PIN TRIGER ARISTERO
#define ECHO_PINL    A1 // echo pin ARISTERO.

#define MAX_DISTANCE 100 // MEGISTI APOSTASI  cm.

#define TRIGGER_PINF  A2//  trigger pin MPROSTA.
#define ECHO_PINF    A3 //  echo pin MPROSTA.

#define TRIGGER_PINR  A4 // trigger pin DEKSIA.
#define ECHO_PINR     A5 // echo pin DEKSIA.





int dir;


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4



float P = 0.7 ;
float D = 0.5 ;
float I = 0.4 ;
float oldErrorP ;
float totalError ;
int offset = 5 ;

int wall_threshold = 15 ;

int front_threshold = 13;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;





int en1 = 7; 
int en2 = 5; 

int en3 = 4 ;
int en4 = 8; 

int enA = 9; 
int enB = 6;

int baseSpeed = 70 ;

int RMS ;
int LMS ;

int LED = 13 ;
int led1 = 10;
int led2 = 11;



NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); 
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 30; // POSO SYXNA UA KANE PING SE  milliseconds  50ms = 20 FORES TO DEYTEROLEPTO.
unsigned long pingTimer;     //KRATA DELAY .


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;

//int TestNUM = 1  ;



void setup() {

  Serial.begin(115200); // ANOIGEI TO SERIAL MONITIOR


  for (int i = 2; i <= 13; i++) //For Motor Shield
    pinMode(i, OUTPUT);



  first_turn = false ;
  rightWallFollow = false ;
  leftWallFollow = false ;
 

}

void loop() {


  //========================================SYNEXES LOOP========================================//


  ReadSensors();

  walls();


  if ( first_turn == false ) {

    pid_start();

  }
  else if (leftWallFollow == true ) {

    PID(true) ;

  }
  else if (rightWallFollow == true ) {
    PID(false) ;
  }


  if (leftwall == true && rightwall == false && frontwall == true ) {

    // turnright();
    PID(false) ;

    if ( first_turn == false ) {

     


      first_turn = true ;
      rightWallFollow = true ;
      
      digitalWrite(led2 , LOW );
      digitalWrite(led1 ,HIGH );
    }
  }
   if (leftwall == false && rightwall == true && frontwall == true ) {

    //  turnleft();
    PID(true) ;

    if ( first_turn == false ) {

     
      first_turn = true ;
      leftWallFollow = true ;
      digitalWrite(LED , HIGH);
       
    }
  }
   if ( leftSensor == 0 || leftSensor > 100 && rightSensor == 0 || rightSensor > 100 && frontSensor == 0 || frontSensor > 100 ) {

    setDirection(STOP);
  }



  // METRISH TVN AISTHTHRON KAI TYPOSI TOYW STO SERIAL MONITOR //


  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");

  //METRISI LATHOYW KAI EKTYPOSH STO SERIAL MONITOR


  Serial.print("error=");
  Serial.println(totalError);


}








//--------------------------------- ELEGXOS STROFON ---------------------------------//

void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, LOW);   //ARISTEROS TROXOS MPROSTA 
    digitalWrite(en2, HIGH);
    digitalWrite(en3, LOW);  // DEKSIOS TROXOS MPROSTA
    digitalWrite(en4, HIGH);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, HIGH);   //ARISTEROS TROXOS MPROSTA 
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW );  //DEKSIOS TROXOS MPROSTA 
    digitalWrite(en4, HIGH);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, LOW);   // ARISTEROS TROXOS MPROSTA
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  // DEKSIOS TROXOS MPROSTA
    digitalWrite(en4, LOW);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1, HIGH);   // ARISTEROS TROXOS MPROSTA
    digitalWrite(en2, HIGH );
    digitalWrite(en3, HIGH );  //DEKSIOS TROXOS MPROSTA 
    digitalWrite(en4, HIGH);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, HIGH );   //ARISTEROS TROXOS MPROSTA
    digitalWrite(en2, LOW );
    digitalWrite(en3, HIGH );  // DEKSIOS TROXOS MPROSTA
    digitalWrite(en4, LOW );
  }
}
//---------------------------------------------------------------------------//


//--------------------------------- Sensors ---------------------------------//

void ReadSensors() {

 

  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();


  leftSensor = (lSensor + oldLeftSensor) / 2; //MESI TIMI APO THN APLIA KAI THN NEA METRHSH
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // KRATAME THN PALIA METRISI
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}

//---------------------------------------------------------------------------//


//--------------------------------- ELEGXOS ---------------------------------//

void pid_start() {

  //ReadSensors()

  float errorP = leftSensor - rightSensor ;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * errorI + errorP ;

  totalError = P * errorP + D * errorD + I * errorI ;

  oldErrorP = errorP ;


  RMS = baseSpeed + totalError ;
  LMS = baseSpeed - totalError ;

 


  if (RMS < 0) {

    RMS = map(RMS , 0 , -255, 0, 255);

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(RIGHT);

  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);


    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(LEFT);
  }
  else {

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(FORWARD);
  }



}


//-----------------------------AKOLOYTHISI TOIXOY -------------------------------//

void PID( boolean left ) {

  if (left == true ) {

    float errorP = leftSensor - rightSensor - offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;


    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    


    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }
  else {

    float errorP = leftSensor - rightSensor + offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;

    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

   
    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }

}

//--------------------------- ANIXNEYSH TOIXOY --------------------------------//

void walls() {


  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}



//----------------------------------STROFI DEKSIA-----------------------------------------//

void turnright() {


  LMS = baseSpeed ;

  RMS = LMS * rightSensor / ( rightSensor + 11 ) ;


}

//----------------------------------STROFI ARISTERA-----------------------------------------//

void turnleft() {


  RMS = baseSpeed ;

  LMS = RMS * leftSensor / ( leftSensor + 11 ) ;

}


//---------------------------------------------------------------------------//




