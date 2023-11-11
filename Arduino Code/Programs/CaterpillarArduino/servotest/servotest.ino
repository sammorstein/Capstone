#include <Servo.h>


Servo myservoL;
Servo myservoR;
Servo myservoP;
Servo myservoT;

void setup() {
  // put your setup code here, to run once:
  myservoL.attach(5);
  myservoR.attach(44);
  myservoP.attach(45);
  myservoT.attach(46);

  Serial.begin(9600);
  Serial.println("test");
}

void loop() {
  // put your main code here, to run repeatedly:
  // myservoL.write(85);
  // myservoR.write(55);
  myservoP.write(90);
  myservoT.write(45);
  myservoL.detach();
  myservoR.detach();

  delay(500);
}
