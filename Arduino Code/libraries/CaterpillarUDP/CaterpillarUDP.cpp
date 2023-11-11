#include "Ethernet.h"
#include "EthernetUdp.h"
#include "CaterpillarUDP.h"
#include "ArduinoJson.h"
  

//default constructor for UDP class. You could change the values here and call this instead of calling the full constructor
//to save some variable memory on the arduino!
CaterpillarUDP::CaterpillarUDP()
{
	byte mac[6] = {0xA8, 0x61, 0x0A, 0xAE, 0xAA, 0x2A};
	byte i = 6;
	while (i--) *(_mac + i) = *(mac + i);
	_ip.fromString("10.137.19.131");
	_localPort = 5020;
	_sizeJSON = 48;
}
//main constructor for UDP class.
//inputs:
//mac - the mac address as an array of bytes
//ip - ip address as string
//localPort - port for UDP server
CaterpillarUDP::CaterpillarUDP(byte *mac, String ip, unsigned int localPort)
{
	byte i = 6;
	while (i--) *(_mac + i) = *(mac + i);
	_ip.fromString(ip);
	_localPort = localPort;
	_sizeJSON = 48;
}
CaterpillarUDP::CaterpillarUDP(byte *mac, String ip, unsigned int localPort, unsigned int sizeJSON)
{
	byte i = 6;
	while (i--) *(_mac + i) = *(mac + i);
	_ip.fromString(ip);
	_localPort = localPort;
	_sizeJSON = sizeJSON;
}
//EthernetSetup sets up the connection with the UDP server. Put this in your setup() function.
void CaterpillarUDP::EthernetUDPSetup()
{
	Ethernet.init(10); // Most Arduino shields
	// start the Ethernet
	Ethernet.begin(_mac, _ip);
	Serial.begin(9600);
	while (!Serial)
	{
		; // wait for serial port to connect. Needed for native USB port only
	}
	if (Ethernet.hardwareStatus() == EthernetNoHardware)
	{
        Serial.println("Ethernet shield was not found.    Sorry, can't run without hardware. :(");
        while (true)
        {
            delay(1); // do nothing, no point running without Ethernet hardware
        }
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
        Serial.println("Ethernet cable is not connected.");
    }

    // start UDP
    _Udp.begin(_localPort);
    Serial.println("udp started!");
}
//Continually polls UDP Server for new data. Put this in your loop() function.
float CaterpillarUDP::UDPServerGetFloat()
{
	char packetBuffer[50];
	int packetSize = 0;
	bool previousPacketIsLoaded = false;
	while (true) {
		packetSize = _Udp.parsePacket();
		if (packetSize) { // raise flag that a packet is loaded and read it in the buffer
			previousPacketIsLoaded = true;
			_Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
		}
		else if (!packetSize && previousPacketIsLoaded) { // if the current packet is empty, but a loaded packet exists, break out of the loop
			previousPacketIsLoaded = false;
			String datReq(packetBuffer);
			float x = datReq.toFloat();
			return x;
		} else {
		}
	}
	return -2;
	/*
	int packetSize = _Udp.parsePacket(); // if there's data available, read a packet
	Serial.print(packetSize);
	if (packetSize)
	{ // if packetSize has a value, ie there is a packet present
		IPAddress remote = _Udp.remoteIP();
		_Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); // read the packet into packetBuffer
		Serial.println(packetBuffer);
		String datReq(packetBuffer);
		float x = datReq.toFloat();
  
		//    Serial.print("The data Req is:");
		//    Serial.println(datReq);
		return x;
	}
	Serial.print("yo");
	return -2;
	*/
	
}
CaterpillarUDP::JsonDocument CaterpillarUDP::UDPServerGetJson()
{
	char packetBuffer[100];
	int packetSize = 0;
	bool previousPacketIsLoaded = false;
	while (true) {
		packetSize = _Udp.parsePacket();
		//Serial.println(packetSize);
		if (packetSize > 0) { // raise flag that a packet is loaded and read it in the buffer
			previousPacketIsLoaded = true;
			_Udp.read(packetBuffer, 100);
		}
		else if (packetSize == 0 && previousPacketIsLoaded) { // if the current packet is empty, but a loaded packet exists, break out of the loop
			StaticJsonDocument<200> doc;
			DeserializationError error = deserializeJson(doc, packetBuffer, 200);
			//serializeJson(doc, Serial);
			//Serial.println();
			return doc;
		} else if (packetSize < 0) {
			//Serial.println("reset port!");
			_Udp.stop();
			_Udp.begin(_localPort);
		}
	}
	
}