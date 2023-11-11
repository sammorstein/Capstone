#include <Servo.h>
#include <CaterpillarUDP.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>

#define ENCAtilt 2 // Yellow
#define ENCBtilt 3 // Green
#define ENCApan 18 // Yellow
#define ENCBpan 19 // Green
#define Go 20 //Run switch

#define rev1 700 //number of ticks per one revolution
#define maxSpeed 150
#define minSpeed -150


//make byte array for mac address so we can pass it to function
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x94, 0xE0
};
//UDP class constructor
CaterpillarUDP test(mac, "169.254.192.103", 5020, 48); //mac, ip, port
StaticJsonDocument<200> doc;


Servo servoPan;
Servo servoTilt;
Servo servoPincerL;
Servo servoPincerR;

// dc tilt motor pins
const int pinPwmTilt = 9;
const int pinDirTilt = 8;

// dc pan motor pins
const int pinPwmPan = 6;
const int pinDirPan = 7;

void setup() {
  pinMode(ENCAtilt,INPUT);
  pinMode(ENCBtilt,INPUT);
  pinMode(ENCApan,INPUT);
  pinMode(ENCBpan,INPUT);
  pinMode(pinPwmTilt, OUTPUT);
  pinMode(pinDirTilt, OUTPUT);
  pinMode(pinPwmPan, OUTPUT);
  pinMode(pinDirPan, OUTPUT);
  pinMode(Go, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCAtilt),readEncoderTilt,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCApan),readEncoderPan,RISING);
  attachInterrupt(digitalPinToInterrupt(Go), Stop,FALLING);
  servoPan.attach(45);
  servoTilt.attach(46);
  // servoPincerL.attach(5);
  // servoPincerR.attach(44);


  Serial.begin(9600);

  test.EthernetUDPSetup(); //setup UDP connection with server
}


  //Caterpillar angles 
  //servos might want to be ints instead
  //pincers code is included but has been commented out due to servo issues
  float torsoPan;
  float torsoTilt;
  float headPan;
  float headTilt;
  // float pincerL;
  // float pincerR;

  //tilt motor parameters
  float kpTilt = 1.5;
  float kiTilt = 1;
  float kdTilt = 0.1;
  float tiltError = 0;
  float posTilt = 0;
  float angTilt = 0;
  float tiltIntegral = 0;
  float tiltImax = maxSpeed/kiTilt;
  float tiltImin = minSpeed/kiTilt;
  float tiltDerivative = 0;
  float tiltSpeed = 0;
  float prevTiltError = 0;

  
  //pan motor parameters
  float kpPan = 0.75;
  float kiPan = 1;
  float kdPan = 0.025;
  float panError = 0;
  float posPan = 0;
  float angPan = 0;
  float panIntegral = 0;
  float panImax = maxSpeed/kiPan;
  float panImin = minSpeed/kiPan;
  float panDerivative = 0;
  float panSpeed = 0;
  float prevPanError = 0;

  //timing
  long prevT = 0;
  long currT = 0;
  float deltaT = 0.1;
  long prevRead = 0;

void loop() {

  //stops loop when switch is off
  while (!digitalRead(Go)) {
    prevT = micros();           //offsets the start time to match when the switch is flipped
    currT = micros();
    prevRead = micros();        //offsets the start time to match when the switch is flipped
    servoPan.write(90);
    servoTilt.write(45);
  }
  
  if((currT-prevRead) > 500000){ //check if its been 0.5 sec since last read

    prevRead = currT;
    //read from UDP
    doc = test.UDPServerGetJson();
    torsoPan = doc["h"]["t1"]; //Pan DC motor angle
    torsoTilt = doc["h"]["t2"]; //Tilt DC motor angle
    headPan= doc["h"]["t3"]; //Pan servo motor 
    headTilt = doc["h"]["t4"]; //Tilt servo motor
    // pincerL = doc["h"]["t5"]; //Left servo motor
    // pincerR = doc["h"]["t6"]; //Right servo motor
    
    //convert kinematics to caterpillar angles
    torsoTilt = torsoTilt + 90;
    headPan = -headPan + 90;
    headTilt = -headTilt + 90;
    // pincerL = pincerL + 100;
    // pincerR = pincerR + 75;

    //sets the angle limits off the motors
    torsoTilt = constrain(torsoTilt, 15, 75);
    torsoPan = constrain(torsoPan, -60, 30);
    headTilt = constrain(headTilt, 30, 90);
    headTilt = constrain(headTilt, 0, 180);
    // pincerL = constrain(pincerL, 100, 135);
    // pincerR = constrain(pincerR, 75, 100);

    Serial.println(torsoPan);
    Serial.println(torsoTilt);
    Serial.println(headPan);
    Serial.println(headTilt);
    Serial.println();

    Serial.print("Tilt Angle ");
    Serial.println(angTilt);
    Serial.print("Pan Angle ");
    Serial.println(angPan);
  }

  pose(torsoTilt,torsoPan,headTilt,headPan); //calls function to move the caterpillar
  
  
    
}
  
void pose(float angT, float angP, int servoT, int servoP) { //moves the caterpillar towards the target position - add pincer parameters to move the pincers.
  
  Serial.println();
  servoPan.write(servoP);
  servoTilt.write(servoT);
  // servoPincerR.write(pincerR);
  // servoPincerL.write(pincerL);
  
  //tilt motor calculations
  prevTiltError = tiltError;
  tiltError = angT - angTilt;
  tiltIntegral = tiltIntegral + tiltError*deltaT;
  tiltIntegral = constrain(tiltIntegral, tiltImin, tiltImax);
  if (deltaT > 0.005){
    tiltDerivative = (tiltError - prevTiltError)/deltaT;
  }
  tiltSpeed = kpTilt*tiltError + kiTilt*tiltIntegral + kdTilt*tiltDerivative;
  tiltSpeed = constrain(tiltSpeed, minSpeed, maxSpeed);
  Serial.print("Tilt Speed ");
  
  // writes to the tilt motor
  if (tiltSpeed > 0) {
    analogWrite(pinPwmTilt, abs(tiltSpeed));
    digitalWrite(pinDirTilt, LOW);
  }
  else {
    analogWrite(pinPwmTilt, abs(tiltSpeed));
    digitalWrite(pinDirTilt, HIGH);
  }
  
  Serial.println(tiltSpeed);

  //pan motor update calculations
  prevPanError = panError;
  panError = angP - angPan;
  panIntegral = panIntegral + panError*deltaT;
  panIntegral = constrain(panIntegral, panImin, panImax);
  if (deltaT > 0.005) {
    panDerivative = (panError - prevPanError)/deltaT;
  }
  Serial.print("Pan Speed ");
  panSpeed = kpPan*panError + kiPan*panIntegral + kdPan*panDerivative;
  panSpeed = constrain(panSpeed, minSpeed, maxSpeed);

  //writes to the pan motor
  if (panSpeed > 0) {
    analogWrite(pinPwmPan, abs(panSpeed));
    digitalWrite(pinDirPan, LOW);
  }
  else {
    analogWrite(pinPwmPan, abs(panSpeed));
    digitalWrite(pinDirPan, HIGH);
  }
  
  Serial.println(panSpeed);
  Serial.println();

  Serial.println("Finished");  

  //time update
  prevT = currT;
  currT = micros();
  if (abs(currT-prevT) < 0.02){ //prevents derivative term from going to infinity
    deltaT = 0.06;
  } else {
    deltaT = ((float) (currT-prevT))/1.0e6;
  }
  Serial.println(deltaT);
}



void readEncoderTilt(){          //updates tilt position
  int b = digitalRead(ENCBtilt);
  if(b > 0){
    posTilt++;
  }
  else{
    posTilt--;
  }
  angTilt = (float)posTilt*360/700;
}


void readEncoderPan(){          //updates pan position  
  int b = digitalRead(ENCBpan);
  if(b > 0){
    posPan++;
  }
  else{
    posPan--;
  }
  angPan = (float)posPan*360/700;
}


void Stop() {                  //occurs when switch is flipped off
  while(!digitalRead(Go)) {
    analogWrite(pinPwmTilt, 0);
    analogWrite(pinPwmPan, 0);
    prevT = micros();
    currT = micros();
    prevRead = micros();
  }
}

