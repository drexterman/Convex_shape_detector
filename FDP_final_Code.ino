#include <SoftwareSerial.h>
int MLspeed = 11; //controlling left wheel's speed              ENB
int MRspeed = 10; //controlling right wheel's speed             ENA
int motorLeft2 = 12; //moving left wheel in reverse direction   in4
int motorRight1 = 9; //moving right wheel in forward direction  in1
int LED  =  13;      //LED to show concave and convex  
#define sensorIRPin 4

int counter=0;
int distanceThresholdUS= 130 ;
int distanceIR=0;
int x=1;
float X=0;
float Y=0;
int Theta=0; //starting angle with X-axis
int alpha=1;    //angle rotated by bot in one loop
float d=10;        //distance travelled by bot in one loop 
SoftwareSerial mySerial(3, 2);   //2 pe trig and 3 pe echo

unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len  = 0;

void setup() {
  
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
  pinMode(motorLeft2,OUTPUT);
  pinMode(motorRight1,OUTPUT);
  pinMode(MLspeed,OUTPUT);
  pinMode(MRspeed,OUTPUT);
  pinMode(sensorIRPin,INPUT);
  mySerial.begin(9600);
}

void loop() {
    digitalWrite(motorRight1, HIGH);
    digitalWrite(motorLeft2, HIGH);


  mySerial.flush();
  mySerial.write(0X55);                           // trig US-100 begin to measure the distance
  delay(500);                                  
  if (mySerial.available() >= 2)                  // check receive 2 bytes correctly
  {
    HighByte = mySerial.read();
    LowByte  = mySerial.read();
    Len  = HighByte * 256 + LowByte;          // Calculate the distance
    if ((Len > 1) && (Len < 10000))
    {
//      Serial.print("Distance: ");
//      Serial.print(Len);          
//      Serial.println("mm");
                       
    }
  }
  else{}
   distanceIR=digitalRead(sensorIRPin);   //reding if there exists an object in front
//   Serial.print("IR value");
//   Serial.println(distanceIR);
//   Serial.print("distanceThresholdUS");
//   Serial.println(distanceThresholdUS);

  if (distanceIR ==0) {                     //if there is an object in front the bot stops
  // We will stop our robot by

  for (int i = 0; i < 5; i++) {
    analogWrite(MLspeed, 0);
    analogWrite(MRspeed, 0);
    delay(5);
}

digitalWrite(LED,HIGH);
    }
  else { 
        digitalWrite(LED,LOW);                                       
    if (Len < 80) {            //if the bot starts coming close to a side it will turn right or move away
    // We will turn away from wall by 
    //turning right
      for (int i = 0; i <1; i++) {
    analogWrite(MLspeed, 200);
    analogWrite(MRspeed, 140);
    delay(1);
}
    Theta=Theta-alpha;                         //angle with X-axis changes
    }
  else if (distanceThresholdUS+400>Len && Len> 170) {         //if the bot starts going away from the it turns left or moves closer
   // We will turn towards the wall by
   //turning left
         for (int i = 0; i < 1; i++) {
    analogWrite(MLspeed, 140);
    analogWrite(MRspeed, 200);
    delay(1);
}
    Theta = Theta + alpha;  
        digitalWrite(LED,LOW);
    //angle with X-axis changes
    }
   else if(Len>distanceThresholdUS+400){  //sharp turn left
    for (int i = 0; i <1; i++) {
    analogWrite(MLspeed, 120);
    analogWrite(MRspeed, 200);
    delay(1);}
    Theta = Theta + 2;

    }
    // We will move forward
    else{                                        //if bot is at constant distance it will move forward 
      Serial.println("hi3");
    for (int i = 0; i < 1; i++) {
    analogWrite(MLspeed, 200);
    analogWrite(MRspeed, 200);
    delay(1);
}
    X=X+d*cos(Theta);
    Y=Y+d*sin(Theta);

    }
 Serial.print(X);
 Serial.print(",");
 Serial.println(Y);
 if(Theta>60){   //we did this to stop out bot after approximately rotating once around the given object
  for (int i = 0; i < 5; i++) {
    analogWrite(MLspeed, 0);
    analogWrite(MRspeed, 0);
    delay(5);
}
exit(0);
  }

    
}
}
