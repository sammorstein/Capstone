#ifndef CaterpillarUDP_h
#define CaterpillarUDP_h

#include "Ethernet.h"
#include "EthernetUdp.h"
#include "ArduinoJson.h"

class CaterpillarUDP
{
public:
	CaterpillarUDP();
	CaterpillarUDP(byte *mac, String ip, unsigned int localPort);
	CaterpillarUDP(byte *mac, String ip, unsigned int localPort, unsigned int _sizeJSON);
	void EthernetUDPSetup();
	float UDPServerGetFloat();
	using JsonDocument = StaticJsonDocument<64>;
	JsonDocument UDPServerGetJson();
private:	
byte _mac[6];
IPAddress _ip;
unsigned int _localPort;
EthernetUDP _Udp;
unsigned int _sizeJSON;
};

#endif