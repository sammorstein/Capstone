#include <CaterpillarUDP.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>
//make byte array for mac address so we can pass it to function
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0xAA, 0x2A
};
//UDP class constructor
CaterpillarUDP test(mac, "10.137.19.131", 5020, 48); //mac, ip, port
StaticJsonDocument<64> doc;
int count = 0;

void setup() {
  test.EthernetUDPSetup(); //setup UDP connection with server
}

void loop() {
  float delayStart = millis();
  //float x = test.UDPServerGetFloat(); //continually poll for new data from UDP server
  doc = test.UDPServerGetJson();
  float x = doc["hw"]["yR"]; // 0
  Serial.print((millis() - delayStart));
  Serial.println(" ms");
  Serial.println(x);
  if (Ethernet.linkStatus() == LinkOFF)
    {
        Serial.println("Ethernet cable is not connected.");
    }
  
}
