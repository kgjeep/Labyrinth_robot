//#include <TimerOne.h>

#include <NewPing.h>

// Πρόγραμμα αυτόνομης κίνησης ρομπότ
#include <Servo.h>


// Σύνδεση του LN298N με τα pin του Arduino
// Κινητήρας 1 - Motor Α (Αριστερός τροχός)
int enA = 6;
int in1 = 7;
int in2 = 5;
int t;
// Κινητήρας 2 – Motor B (Δεξιός τροχός)
int enB = 9;
int in3 = 4;
int in4 = 8;
// Έλεγχος Αποστασιόμετρου
const int trigPinL = A0;
const int echoPinL = A1;
const int trigPinC = A2;
const int echoPinC = A3;
const int trigPinR = A4;
const int echoPinR = A5;
#define MaxDistance 2000
NewPing sonarL(trigPinL,echoPinL,MaxDistance);
NewPing sonarC(trigPinC,echoPinC,MaxDistance);
NewPing sonarR(trigPinR,echoPinR,MaxDistance);


long cm_dist_C, fSensor, oldFrontSensor;
long cm_dist_L, lSensor, oldLeftSensor;
long cm_dist_R, rSensor, oldRightSensor;
long cm_dist_L_First;
long left_sonar;
long error ,last_error,derivative,integral;
// Constants for Interrupt Pins
// Change values if not using Arduino Uno
const byte MOTOR_A = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk, change if different
 
// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

// times gia calibration ton taxytiton ton  troxon
const float A_agjst= 1.15; 
const float B_agjst= 0.85; 
// Integers for pulse counters
volatile int counter_A = 0;
volatile int counter_B = 0;
// Interrupt Service Routines
 
// Motor A pulse count ISR
void ISR_countA()  
{
  counter_A++;// increment Motor A counter value counter_A = counter_A + 1
   //time_a=millis();
} 
 
// Motor B pulse count ISR
void ISR_countB()  
{
  counter_B++;  // increment Motor B counter value
  //time_b=millis();
}
// Function to convert from centimeters to steps
int CMtoSteps(float cm) {
 
  int result;  // Final calculation result
  float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result
 
}
 
// Function to Move Reverce
void MoveReverse(int steps, int mspeed) 
{
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   
   // Set Motor A forward
   digitalWrite(in1, HIGH);
   digitalWrite(in2, LOW);
 
   // Set Motor B forward
   digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);
   
   // Go forward until step value is reached
   while (steps > counter_A || steps > counter_B) {
   
    if (steps >= counter_A) {
    analogWrite(enA, mspeed*A_agjst);
    } else {
    analogWrite(enA, 0);
    }
    if (steps >= counter_B) {
    analogWrite(enB, mspeed*B_agjst);
    } else {
    analogWrite(enB, 0);
    }
   }
  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  Serial.print("Back  Count_A= ");
   Serial.print(counter_A);
   Serial.print("   Back  Count_B= ");
   Serial.print(counter_B);
   Serial.println();
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
 
}
 
// Function to Move in Forward
void MoveForward(int steps, int mspeed) 
{
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   
   // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
   
   // Go in reverse until step value is reached
   while (steps > counter_A && steps > counter_B) {
    left_sonar = sonarL.ping_cm();

    error = cm_dist_L_First - left_sonar;
    last_error = error;
    derivative = error - last_error;
     if(abs(error) < 0.2)
  {
    integral = integral + error;
  }
  else
  {
    integral=0;
  }
    if (steps >= counter_A) {
    analogWrite(enA, mspeed*A_agjst);// + (error*0.7+integral*0.05+derivative*1.3));
    } else {
    analogWrite(enA, 0);
    
    }
    if (steps >= counter_B) {
    analogWrite(enB, mspeed*B_agjst);// + (error*0.7+integral*0.05+derivative*1.3));
    } else {
    analogWrite(enB, 0);
    
    }
    /*if (left_sonar > cm_dist_L_First){
      analogWrite(enB, 0);
      delay(10);
      analogWrite(enB, mspeed*B_agjst);
    }
     if (left_sonar < cm_dist_L_First){
      analogWrite(enA, 0);
      delay(10);
      analogWrite(enA, mspeed*A_agjst);
    }*/
    }
    
  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  Serial.print(" A Counts: ");
    Serial.print(counter_A);
    Serial.print(" ticks ");
    Serial.print(" B Counts: ");
    Serial.print(counter_B);
    Serial.println(" ticks ");
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
 
}
 
// Function to Spin Left
void SpinLeft(int steps, int mspeed) 
{
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   
   // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
   
   // Go until step value is reached
   while (steps > counter_A || steps > counter_B) {
   
    if (steps >= counter_A) {
    analogWrite(enA, mspeed*A_agjst);
    } else {
    analogWrite(enA, 0);
    }
    if (steps >= counter_B) {
    analogWrite(enB, mspeed*B_agjst);
    } else {
    analogWrite(enB, 0);
    }
   }
    
  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
 
}
 
// Function to Spin Right
void SpinRight(int steps, int mspeed) 
{
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   
   // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
 
  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
   
   // Go until step value is reached
   while (steps > counter_A || steps > counter_B) {
   
    if (steps >= counter_A) {
    analogWrite(enA, mspeed*A_agjst);
    } else {
    analogWrite(enA, 0);
    }
    if (steps >= counter_B) {
    analogWrite(enB, mspeed*B_agjst);
    } else {
    analogWrite(enB, 0);
    }
  }
    
  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
 
}
 

// Η συνάρτηση setup εκτελείται μία φορά κατά την έναρξη του προγράμματος
void setup() {
Serial.begin(9600);
// Attach the Interrupts to their ISR's

attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High

// Ορισμός όλων των pin ελέγχου των κινητήρων και του αποστασιόμετρου ως εξόδων
pinMode(enA, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);
pinMode(trigPinL, OUTPUT);
pinMode(echoPinL, INPUT);
pinMode(trigPinC, OUTPUT);
pinMode(echoPinC, INPUT);
pinMode(trigPinR, OUTPUT);
pinMode(echoPinR, INPUT);
delay(2000);
cm_dist_L_First=sonarL.ping_cm();
 
}
void ReadSensors() {
 cm_dist_L = sonarL.ping_cm(); //ping in cm
 while (cm_dist_L== 0){
     cm_dist_L = sonarL.ping_cm();
  }
  //delay(33);
    cm_dist_C = sonarC.ping_cm();
    while (cm_dist_C== 0){
     cm_dist_C = sonarC.ping_cm();
  }
 //delay(33);
  cm_dist_R = sonarR.ping_cm();
  while (cm_dist_R== 0){
     cm_dist_R = sonarR.ping_cm();
  }
  
  /*
    Serial.print(" Left : ");
  Serial.print(cm_dist_L);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(cm_dist_C);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(cm_dist_R);
  Serial.println(" cm ");
  */
   }


// Η συνάρτηση loop εκτελείται συνέχεια ως ατέρμων βρόχος

void loop() {
  // put your main code here, to run repeatedly:

   ReadSensors();

    if ((cm_dist_L < 20) && (cm_dist_C < 20)&& (cm_dist_R <20)){
    SpinRight(14, 110);
    delay(1000);
    }
       else if ((cm_dist_L < 20) && (cm_dist_C < 20)&& (cm_dist_R >= 20)){
       MoveForward(CMtoSteps(10),120);
       SpinRight(6, 110);
       MoveForward(CMtoSteps(26),120);
       delay(1000);
       }
         else if ((cm_dist_L < 20) && (cm_dist_C >= 20)){
         MoveForward(4,120);
         }
          else if (cm_dist_L >= 30 && cm_dist_L != 0){
          MoveForward(CMtoSteps(10),120);
          delay(1000);
          SpinLeft(6, 110);
          delay(1000);
          MoveForward(CMtoSteps(26),120);
          delay(1000);
          SpinLeft(6, 110);
          delay(1000);
          MoveForward(CMtoSteps(10),120);
          delay(1000);
   }
   /*
          //SpinRight(13, 100);
          //SpinRight(6, 100);
          //SpinLeft(6, 100);
          //delay(1000);
           while (t<10) {
    t=t+1;
  
     MoveForward(4,120);
     //delay(1000);
   }
   */
}
