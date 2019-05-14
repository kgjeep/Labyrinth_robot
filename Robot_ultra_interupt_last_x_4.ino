// Πρόγραμμα αυτόνομης κίνησης ρομπότ
#include <Servo.h>
#include <NewPing.h>
#include "TimerOne.h"
// Σύνδεση του LN298N με τα pin του Arduino
// Κινητήρας 1 - Motor Α (Δεξιός τροχός)
int enA = 6;
int in1 = 7;
int in2 = 5;
// Κινητήρας 2 – Motor B (Αριστερός τροχός)
int enB = 9;
int in3 = 4;
int in4 = 8;
//Ορισμός αντικειμένου κλασης Servo
Servo myservo;
int pin_out =1;
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


long cm_dist_C;
long cm_dist_L;
long cm_dist_R;

// Constants for Interrupt Pins
// Change values if not using Arduino Uno
 
const byte MOTOR_B = 2;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_A = 3;  // Motor 1 Interrupt Pin - INT 0 - Left Motor
 
// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk, change if different
 
// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different
 
// Integers for pulse counters
volatile int counter_A = 0;
volatile int counter_B = 0;
volatile float time_a,time_b,time_a_old,time_b_old;
//volatile float error;
// Interrupt Service Routines
 
// Motor A pulse count ISR
void ISR_countA()  
{
  counter_A++;// increment Motor A counter value counter_A = counter_A + 1
   time_a=millis();
} 
 
// Motor B pulse count ISR
void ISR_countB()  
{
  counter_B++;  // increment Motor B counter value
  time_b=millis();
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
   
   // Go Backwards orward until step value is reached
    while (steps > counter_A || steps > counter_B) {
   analogWrite(enA, mspeed*1.12);
   analogWrite(enB, mspeed*0.88);
    if (steps <= counter_A) {
    analogWrite(enA, 0);
    } 
    
    if (steps <= counter_B) {
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
  float error=0;
  float rotationA;
  float rotationB;
   // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
   
   // Go in reverse until step value is reached
   delay(33);
   cm_dist_C=sonarC.ping_cm();
   analogWrite(enA, mspeed);
    analogWrite(enB, mspeed);
    delay(100);
   while ( cm_dist_C > 20||cm_dist_C == 0 ){
  /* analogWrite(enA, mspeed*1.12);// me idies taxythtew stribei aristera
   // gia na paei isia prepei o A na exei megaliteri timh apo to b 
   //oso megalyteri h diafora stribei deksia
   analogWrite(enB, mspeed*.89);
   */
   time_a_old=time_a;
    time_b_old=time_b;
    delay(33);
   cm_dist_C=sonarC.ping_cm();
   
    rotationA = (1./(millis()-time_a_old)) * 60000.00;  // calculate RPM for Motor 1
    rotationB = (1./(millis()-time_b_old)) * 60000.00;  // calculate RPM for Motor 2
  error= rotationA-rotationB;
   
    analogWrite(enA, mspeed);
    analogWrite(enB, mspeed+error*.1);
    Serial.print("mspeed = ");
    Serial.print(mspeed);
    Serial.print("  error = ");
    Serial.print(error);
    Serial.print("  mspeed +error= ");
    Serial.print(mspeed+error*0.1);
    Serial.println();
    /*if (steps <= counter_A) {
    //analogWrite(enA, 0);
    time_a=millis();
    } 
    
    if (steps <= counter_B) {
    //  analogWrite(enB, 0);
     time_b=millis();
    }*/
   
 
  Serial.print("Rotation A: "); 
  Serial.print(rotationA);  
  Serial.print(" RPM - ");
   Serial.print("Rotation B: "); 
  
  Serial.print(rotationB);  
  Serial.println(" RPM");
  // Stop when done
 
  //analogWrite(enA, 0);
 //analogWrite(enB, 0);
  Serial.print("Forw  Count_A= ");
   Serial.print(counter_A);
   Serial.print("    Forw  Count_B= ");
   Serial.print(counter_B);
   Serial.println();
   
 counter_A = 0;  //  reset counter A to zero ean ta exo frenarei kai paei diakekomena
  counter_B = 0;  //  reset counter B to zero 
}
 
 
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
    analogWrite(enA, mspeed*1.12);
    analogWrite(enB, mspeed*0.88);
    if (steps <= counter_A) {
     analogWrite(enA, 0);
    }
    if (steps <= counter_B) {
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
   analogWrite(enA, mspeed*1.12);
   analogWrite(enB, mspeed*0.88);
    if (steps <= counter_A) {
    analogWrite(enA, 0);
    } 
    
    if (steps <= counter_B) {
        analogWrite(enB, 0);
    }
    }
       
  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
 
}

/* TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt();  // Stop the timer
  Serial.print("Motor Speed A: "); 
  float rotationA = (counter_A/ stepcount) * 60.00;  // calculate RPM for Motor 1
  Serial.print(rotationA);  
  Serial.print(" RPM - "); 
  counter_A= 0;  //  reset counter to zero
  Serial.print("Motor Speed B: "); 
  float rotationB = (counter_B / stepcount) * 60.00;  // calculate RPM for Motor 2
  Serial.print(rotationB);  
  Serial.println(" RPM"); 
  counter_B = 0;  //  reset counter to zero
  Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
}
*/

// Η συνάρτηση setup εκτελείται μία φορά κατά την έναρξη του προγράμματος
void setup() {
Serial.begin(9600);
// Attach the Interrupts to their ISR's
//Timer1.initialize(1000000); // set timer for 1sec
attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High
//Timer1.attachInterrupt( ISR_timerone ); // Enable the timer 
//cm_dist_L_first=sonarL.ping_cm();


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
delay(1000);
}


// Η συνάρτηση loop εκτελείται συνέχεια ως ατέρμων βρόχος

void loop() {
  // put your main code here, to run repeatedly:
delay(33);
cm_dist_L=sonarL.ping_cm();
delay(33);
cm_dist_C=sonarC.ping_cm();
delay(33);
cm_dist_R=sonarR.ping_cm();

    
  if (cm_dist_C > 20||cm_dist_C == 0 )
      { MoveForward(4, 120);
       
       }
  else if ( 0<cm_dist_C <= 20 )
      { analogWrite(enA, 0);
        analogWrite(enB, 0);
        delay(500);
        SpinRight(10, 80);
      }

}

