int speedPin_M1 = 5;     //9M1 Speed Control
int speedPin_M2 = 6;     //M2 Speed Control
int directionPin_M1 = 4;     //M1 Direction Control
int directionPin_M2 = 7;     //M1 Direction Control

const int trigPin = 8; //Init pin capteur
const int echoPin = 9; //Init pin capteur

int compteur =0;
void setup(){
  /* Initialise le port série */
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
 
}
long duration, distance;

void loop(){
  if (Serial.available() > 0){
    char data;
    data = Serial.read();
    Serial.print("Hello, you sent me: ");
    Serial.println(data);
    advance(data);
  }

//Capteur ultrason
  if (compteur == 50){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);  
    duration = pulseIn(echoPin, HIGH);
    distance = duration/58.9;
    if(distance<20){
      carStop();
      delay(1500);
    }
    compteur=0;
  }
  compteur =compteur+1;
}

void advance(char data) {
    if (data=='F'){
      carAdvance(100,100);
    }
    if (data=='S'){
      carStop();
      delay(1500);
    }
    if (data=='R'){
      carTurnRight(150,150);
    }
    if (data=='L'){
      carTurnLeft(150,150);
    }
    if (data=='D'){ // Doucement = ralentir
      carAdvance(80,80);
      delay(1500);
    }
}


void carStop(){                 //  Motor Stop
  digitalWrite(speedPin_M2,0);
  digitalWrite(directionPin_M1,LOW);
  digitalWrite(speedPin_M1,0);
  digitalWrite(directionPin_M2,LOW); 
}

void carAdvance(int leftSpeed,int rightSpeed){ //Move backward
  analogWrite (speedPin_M2,leftSpeed);              //PWM Speed Control
  digitalWrite(directionPin_M1,HIGH);
  analogWrite (speedPin_M1,rightSpeed);
  digitalWrite(directionPin_M2,HIGH);
}

void carBack(int leftSpeed,int rightSpeed){       //Move forward
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,LOW);
  analogWrite (speedPin_M1,rightSpeed);
  digitalWrite(directionPin_M2,LOW);
}

void carTurnRight(int leftSpeed,int rightSpeed){      //Turn Left
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,LOW);
  analogWrite (speedPin_M1,rightSpeed);
  digitalWrite(directionPin_M2,HIGH);
}
void carTurnLeft(int leftSpeed,int rightSpeed){      //Turn Right
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,HIGH);
  analogWrite (speedPin_M1,rightSpeed);
  digitalWrite(directionPin_M2,LOW);
}
