// Πρόγραμμα αυτόνομης κίνησης ρομπότ
#include <Servo.h>
#include <NewPing.h>
// Σύνδεση του LN298N με τα pin του Arduino
// Κινητήρας 1 - Motor Α (Δεξιός τροχός)
int enA = 6;
int in1 = 7;
int in2 = 5;
// Κινητήρας 2 – Motor B (Αριστερός τροχός)
int enB = 3;
int in3 = 4;
int in4 = 2;
//Ορισμός αντικειμένου κλασης Servo
Servo myservo;
int pin_out =10;
// Έλεγχος Αποστασιόμετρου
const int trigPin = 9;
const int echoPin = 8;
#define MaxDistance 200
NewPing sonar(trigPin,echoPin,MaxDistance);
// Απόσταση σε εκατοστά
long velA;
long velB;
long diff;
long dist;
long cm_dist;
long cm_dist_left;
long cm_dist_right;
//Θεση του servo μπροστά =90 αριστερά = 45 δεξια= 135
long int posCenter=90.0;
long int posLeft=180.0;
long int posRight=1.0;
// Η συνάρτηση setup εκτελείται μία φορά κατά την έναρξη του προγράμματος
void setup() {
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
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
//ορισμός της θέσης του servo
myservo.write(posLeft);
delay(1000);
}


void Follow_left()
{ 
  //dist=Read_Distance();
  dist=sonar.ping_cm();
  dist=long(dist);
  Serial.print("dist");
  Serial.print(dist);
  //Serial.println();
  diff=15.0-dist;
   Serial.print("  diff= ");
   Serial.print(diff);
    velA=160-8*diff;
     if (velA < 0.0)
     {velA=0.0;}
     else
     {velA=velA;}
    velB=160+8*diff;
     if (velB < 0.0)
     {velB=0.0;}
     else
     {velB=velB;}
    Serial.print("  velA= ");
    Serial.print(velA);
    Serial.print("  velB= ");
    Serial.print(velB);
    Serial.println();
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
  delay(100);
  // Σταμάτημα των κινητήρων
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


// Η συνάρτηση loop εκτελείται συνέχεια ως ατέρμων βρόχος

void loop() {
  
  // put your main code here, to run repeatedly:
  Follow_left();
 
}
