#include <ArtriumServo.h>
#include <WormServo.h>
#include <Servo.h>
#include <CaterpillarUDP.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <WormMotorShield.h>

byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x94, 0xE0
};
WormLED led(2, 3);
WormServo servoBackX;
WormServo servoBackY;
WormServo servoFrontX;
WormServo servoFrontY;
WormServo servoEyes;
WormUDP wormUDP(mac, "10.137.19.150", 10012);
//WormUDP lidarUDP(mac, "10.137.19.132", 20020);
WormMotorShield DC(3);
bool wormOut = false;
int intrsn = 0;
bool occ = false;
float timeForwardTotal = 10500.0f;
float timeBackwardTotal = 10000.0f;
float timeOutMil = 0.0f;
float servoBlinkTime = 0.0f;
float servoBlinkCounter = 0.0f;
bool blinkStart = true;
float xB = 0.0f;
float xF = 0.0f;
float yB = 0.0f;
float yF = 0.0f;
int colorLED = 0;

void setup() {
  wormUDP.EthernetUDPSetup();
  //lidarUDP.EthernetUDPSetup();
  servoBackX.attach(9);
  servoBackY.attach(6);
  servoFrontX.attach(7);
  servoFrontY.attach(5);
  servoEyes.attach(4);
  led.begin();
  DC.begin();
  //Serial.begin(9600);
  //led.setColor(0, 255, 0, 255);
  //led.setColor(1, 255, 0, 255);
  randomSeed(analogRead(0));
  servoBackX.move(90);
  servoBackY.move(90);
  servoFrontX.move(90);
  servoFrontY.move(90);
  servoEyes.move(80);
}

void loop() {
  //DC.run(255, BACKWARD);
  Serial.println(count);
  wormLogicLoop();
  //WormTest();
  //WormServoTest();
}

void WormServoTest() {
  servoBackX.move(90);
  servoFrontX.move(90);
  servoBackY.move(90);
  servoFrontY.move(90);
  /*
  Serial.println(90);
  delay(2000);
  servoBackX.move(50);
  servoFrontX.move(140);
  servoBackY.move(140);
  servoFrontY.move(140);
  Serial.println(180);
  delay(2000);
  servoBackX.move(90);
  servoFrontX.move(90);
  servoBackY.move(90);
  servoFrontY.move(90);
  Serial.println(90);
  delay(2000);
  servoBackX.move(140);
  servoFrontX.move(50);
  servoBackY.move(50);
  servoFrontY.move(50);
  Serial.println(0);
  delay(2000);
  */
}

void WormTest() {
  servoBackX.move(90);
  servoBackY.move(40);
  servoFrontX.move(90);
  servoFrontY.move(90);
  delay(50);
  timeOutMil = millis();
  while (abs(timeOutMil - millis()) < timeForwardTotal) {
    DC.run(255, FORWARD);
  }
  DC.run(0, FORWARD);
  delay(100);
  led.setColor(0, 255, 0, 255);
  led.setColor(1, 255, 0, 255);
  delay(1000);
  servoEyes.move(0);
  servoEyes.move(80);
  for (float i = -1.0f; i <= 0.0f; i = i + .01f) {
    servoBackX.WormDiscMove(-i);
    //servoBackY.WormDiscMove(-i);
    servoFrontX.WormDiscMove(i);
    //servoFrontY.WormDiscMove(i);
    delay(10);
  }
  for (float i = 0.0f; i <= 1.0f; i = i + .01f) {
    servoBackX.WormDiscMove(-i);
    //servoBackY.WormDiscMove(-i);
    servoFrontX.WormDiscMove(i);
    //servoFrontY.WormDiscMove(i);
    delay(10);
  }
  delay(1000);
  servoBackX.move(90);
  servoBackY.move(40);
  servoFrontX.move(90);
  servoFrontY.move(90);
  timeOutMil = millis();
  while (abs(timeOutMil - millis()) < timeBackwardTotal) {
    DC.run(255, BACKWARD);
  }
  DC.run(0, BACKWARD);
  delay(100);
  led.setColor(0, 0, 0, 0);
  led.setColor(1, 0, 0, 0);
  delay(1000);
  servoEyes.move(0);
  servoEyes.move(80);
  delay(1000);
}

void wormLogicLoop() {
  //get servo data and check for occupancy
  Serial.println("before");
  StaticJsonDocument<64> doc;
  doc = wormUDP.UDPServerGetJson();
  Serial.println("after");
  if (doc != NULL) {
    xB = doc["xB"];
    xF = doc["xF"];
    yB = doc["yB"];
    yF = doc["yF"];
    if (xB > 5) {
      occ = false;
    } else {
      occ = true;
    }
  }
  Serial.println("after");
  if (occ && wormOut) {
    Serial.println("xB");
    servoBackX.WormDiscMove(-xB);
    Serial.println("yB");
    servoBackY.WormDiscMove(-yB);
    Serial.println("xF");
    servoFrontX.WormDiscMove(xF);
    Serial.println("yF");
    servoFrontY.WormDiscMove(yF);
    Serial.println("done");
    /*time to start a new blink servo timer! set blinkStart to false and get the servoBlinkTime
    if (blinkStart) {
      servoBlinkTime = random(6000, 10000);
      blinkStart = false;
      servoBlinkCounter = millis();
      Serial.println(servoBlinkTime);
    }
    if we have exceeded blink time, make fred blink and set blinkStart to true
    if ((abs(servoBlinkCounter - millis()) > servoBlinkTime)) {
      Serial.println("blink!");
      servoEyes.move(0);
      servoEyes.move(80);
      blinkStart = true;
    }*/
  //if area is occupied but worm is not out of tree, set him to default servo position
  } else if (occ && !wormOut) {
    StaticJsonDocument<64> docOut = wormUDP.UDPServerGetJson();
    if (docOut != NULL) {
    xB = docOut["xB"];
      if (xB > 5) {
        occ = false;
        wormOut = false;
      } else {
        servoBackX.move(90);
    servoBackY.move(40);
    servoFrontX.move(90);
    servoFrontY.move(90);
    //set LEDs
    colorLED = random(1, 8);
    //colorLED = 3;
    switch (colorLED)
    {
    case 1:
      led.setColor(0, 255, 0, 0); //red
      led.setColor(1, 255, 0, 0);
      break;
    case 2:
      led.setColor(0, 0, 255, 0); //green
      led.setColor(1, 0, 255, 0);
      break;
    case 3:
      led.setColor(0, 0, 0, 255); //blue
      led.setColor(1, 0, 0, 255);
      break;
    case 4:
      led.setColor(0, 255, 255, 0); //yellow
      led.setColor(1, 255, 255, 0);
      break;
    case 5:
      led.setColor(0, 0, 255, 255); //light blue
      led.setColor(1, 0, 255, 255);
      break;
    case 6:
      led.setColor(0, 255, 0, 255); //purple
      led.setColor(1, 255, 0, 255);
      break;
    case 7:
      led.setColor(0, 255, 255, 255); //white
      led.setColor(1, 255, 255, 255);
      break;
    }
    //run DC motor forward for 10.5 seconds
    timeOutMil = millis();
    while (abs(timeOutMil - millis()) < timeForwardTotal) {
    DC.run(255, FORWARD);
    }
    DC.run(0, FORWARD);
    //Fred is now out, so set blinkStart and wormOut to true
    wormOut = true;
    blinkStart = true;
      }
    }

  //if area is not occupied but worm is out, set to default position and drive worm back into tree
  } else if (!occ && wormOut) {
    //double check
    //Serial.println("yo");
    StaticJsonDocument<64> docIn = wormUDP.UDPServerGetJson();
    if (docIn != NULL) {
    xB = docIn["xB"];
      if (xB > 5) {
        servoBackX.move(90);
        servoBackY.move(40);
        servoFrontX.move(90);
        servoFrontY.move(90);
        timeOutMil = millis();
        while (abs(timeOutMil - millis()) < timeBackwardTotal) {
          DC.run(255, BACKWARD);
        }
        DC.run(0, BACKWARD);
        led.setColor(0, 0, 0, 0);
        led.setColor(1, 0, 0, 0);
        servoBackX.move(90);
        servoBackY.move(90);
        servoFrontX.move(90);
        servoFrontY.move(90);
        wormOut = false;
      } else {
        occ = true;
        wormOut = true;
      }
    }
  }
}
