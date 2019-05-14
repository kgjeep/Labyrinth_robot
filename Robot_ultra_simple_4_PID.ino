// Πρόγραμμα αυτόνομης κίνησης ρομπότ
#include <Servo.h>
#include <NewPing.h>
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
// Απόσταση σε εκατοστά
long velA;
long velB;
long diff;
long last_diff;
long dist;
long error;
long integral;
long derivative;
long last_dist;
long cm_dist_L_first;
long cm_dist_C;
long cm_dist_L;
long cm_dist_R;
unsigned int last_minimumR;
unsigned int minimumR;
unsigned int last_minimumL;
unsigned int minimumL;
//Θεση του servo μπροστά =90 αριστερά = 45 δεξια= 135
//long int posCenter=90.0;
//long int posLeft=180.0;
//long int posRight=1.0;
// Η συνάρτηση setup εκτελείται μία φορά κατά την έναρξη του προγράμματος
void setup() {
   cm_dist_L_first=sonarL.ping_cm();
delay(33);
Serial.begin(9600);
// ορισμός της εξόδου του Servo
myservo.attach(pin_out);
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
//ορισμός της θέσης του servo
//myservo.write(posCenter);
delay(1000);
}
/*Υπορουτίνα κίνησης προς τα πίσω
void Move_Back()
{
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 120);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 120);
// καθυστέρηση για κίνηση προς τα πίσω
delay(500);
// Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
} */

void Turn_Litle_Right()
{
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 220);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 180);
// καθυστέρηση για κίνηση προς τα πίσω
delay(50);
// Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}

void Turn_Litle_Left()
{
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 180);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 220);
// καθυστέρηση για κίνηση προς τα πίσω
delay(50);
// Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}

// Υπορουτίνα κίνησης προς τα μπροστά
void Move_Forward()
{
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
/* Ορισμός ταχύτητας κινητήρα δεξιού 1 – τιμές 0-255
analogWrite(enA, 210);
// Ορισμός ταχύτητας κινητήρα αριστερού 2 – τιμές 0-255
analogWrite(enB, 140); */
analogWrite(enA, 190);
// Ορισμός ταχύτητας κινητήρα αριστερού 2 – τιμές 0-255
analogWrite(enB, 170);
// καθυστέρηση για κίνηση προς τα εμπρός
delay(100);
// Σταμάτημα των κινητήρων
//digitalWrite(in1, LOW);
//digitalWrite(in2, LOW);
//digitalWrite(in3, LOW);
//digitalWrite(in4, LOW);
}

void Follow_left()
{ 
  
 dist=sonarL.ping_cm();
  
//  Serial.print("dist = ");
//  Serial.print(cm_dist_L);
  dist=long(dist);
  if (dist == 0.0)
   dist= last_dist+1.0;
  Serial.print("  dist long= ");
  Serial.print(dist);
  last_dist = dist;
  //last_diff=diff;
  diff= 15.- dist;
  
  integral=integral+diff;
  derivative=diff-last_diff;
  last_diff=diff;
  
  Serial.print("  diff = ");
  Serial.print(diff);
  Serial.println();
     velA=190.0+2.*diff+.0*integral+0.1*derivative;
     if (velA < 0.0)
     {velA=0.0;}
     if (velA > 255.)
     {velA=255.;}
     else
     {velA=velA;}
     velB=170.0-2.0*diff-.0*integral-0.1*derivative;
     if (velB < 0.0)
     {velB=0.0;}
     if (velB > 255.)
     {velB=255.;}
     else
     {velB=velB;}
    Serial.print("  velA= ");
    Serial.print(velA);
    Serial.print("  velB= ");
    Serial.print(velB);
    Serial.println();
 
   /*
    if (dist > 15)
    { Turn_Litle_Left();
    //velA=0  ;
      //velB=200 ;
    }
     else if (dist < 15)
    { Turn_Litle_Right();
    //velA=200 ;
      //velB=0 ; 
    }
    else
    velA=160 ;
      velB=160 ;
      */
    // Εκκίνηση του κινητήρα 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // Εκκίνηση του κινητήρα 2
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // Ορισμός ταχύτητας κινητήρα δεξιού 1 – τιμές 0-255
  analogWrite(enA, velA);
  // Ορισμός ταχύτητας κινητήρα αριστερού 2 – τιμές 0-255
  analogWrite(enB, velB);
  // καθυστέρηση για κίνηση προς τα εμπρός
  delay(50);
  /*Σταμάτημα των κινητήρων
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  */
}
// Υπορουτίνα επιστροφής προς τα πίσω
void Turn_Back()
{
        digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Εκκίνηση του κινητήρα 2
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      // Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
      analogWrite(enA, 200);
      // Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
      analogWrite(enB, 160);
      // καθυστέρηση για υλοποίηση στροφής
      delay(600);
      // Σταμάτημα των κινητήρων
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
  error=abs(29.0-(cm_dist_L+cm_dist_R));
  while (error > 0.1)
  {
    cm_dist_L=sonarL.ping_cm();
    cm_dist_R=sonarR.ping_cm();
   error=abs(29.0-(cm_dist_L+cm_dist_R));
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 200);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 160);
// καθυστέρηση για υλοποίηση στροφής

delay(80);
// Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
}

// Υπορουτίνα στροφής αριστερά
void Turn_Right()
{
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 200);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 200);
// καθυστέρηση για υλοποίηση στροφής
delay(430);
// Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}


// Υπορουτίνα στροφής δεξιά
void Turn_Left()
{
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 200);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 200);
// καθυστέρηση για υλοποίηση στροφής
delay(430);
// Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}

void Straighten_Up_R()
{
  
  //last_minimumR = 11.0;
// Εκκίνηση του κινητήρα 1
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 160);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 160);
// καθυστέρηση για υλοποίηση στροφής
delay(10);
while (minimumR<=last_minimumR+50)
{
  last_minimumR=sonarC.ping();
  if (last_minimumR==0)
   {last_minimumR=minimumR;}
  Serial.print("  last_minimumR = ");
   Serial.print(last_minimumR);
  delay(30);
  minimumR =sonarC.ping();
  if (minimumR==0)
   {minimumR=last_minimumR;}
  Serial.print("  minimumR = ");
   Serial.print(minimumR);
   Serial.println();
   delay(30);
  }
// Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);

}
void Straighten_Up_L()
{
  //last_minimumL=11.;
  //minimumL=11.;
  // Εκκίνηση του κινητήρα 1
  
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
// Εκκίνηση του κινητήρα 2
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
// Ορισμός ταχύτητας κινητήρα 1 – τιμές 0-255
analogWrite(enA, 120);
// Ορισμός ταχύτητας κινητήρα 2 – τιμές 0-255
analogWrite(enB, 120);
// καθυστέρηση για υλοποίηση στροφής
while (minimumL <= last_minimumL+50)
{
  last_minimumL=sonarC.ping();
  if (last_minimumL==0)
   {last_minimumL=minimumL;}
  Serial.print("  last_minimumL = ");
   Serial.print(last_minimumL);
  delay(30);
  minimumL =sonarC.ping();
  if (minimumL==0)
   {minimumL=last_minimumL;}
  Serial.print("  minimumL = ");
   Serial.print(minimumL);
   Serial.println();
   delay(30);
}
//Σταμάτημα των κινητήρων
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);

}
// Η συνάρτηση loop εκτελείται συνέχεια ως ατέρμων βρόχος

void loop() {
 
   Main:
  // put your main code here, to run repeatedly:

cm_dist_L=sonarL.ping_cm();
delay(33);
cm_dist_C=sonarC.ping_cm();
delay(33);
cm_dist_R=sonarR.ping_cm();
   Serial.print("  Left Sonar= ");
   Serial.print(cm_dist_L);
   Serial.print("  Center_Sonar= ");
   Serial.print(cm_dist_C);
   Serial.print("  Right_Sonar= ");
   Serial.print(cm_dist_R);
   Serial.println();
 
  // Turn_Right();
   //delay(5000);
   //Turn_Left();
  //delay(5000);
  // Move_Forward();
   //Follow_left();
   
   
   
  
  //  goto Main;  

    
  if (cm_dist_C > 10 && cm_dist_L < 30 )
      { Follow_left();
       goto Main;
       }
   else if (cm_dist_C < 10 && cm_dist_R > 20 && cm_dist_L < 20)
      { Straighten_Up_R();
   Straighten_Up_L();
   Turn_Right();
      Move_Forward();
       Move_Forward();
      goto Main;
      }
   else if (cm_dist_C < 10 && cm_dist_R < 20 && cm_dist_L > 20)
      { Turn_Left();
      Move_Forward();
       Move_Forward();
       goto Main;
      }
   else if (cm_dist_C < 10 && cm_dist_R < 20 && cm_dist_L < 20)
   {Turn_Back();
   delay(5000);
      goto Main;
   }
  
}
