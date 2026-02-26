#include "config.h"

// Sketch -> Include Library -> Manage libraries
// ESP8266WiFi
// PubSubClient by Nich O'Leary

// WifiEsp by bportaluri
// Wifi by arduino
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h>

ESP8266WiFiMulti wifiMulti;
WiFiClient wifiClient;

String ip = "IP not set";

// Выцепить последние два байта из MAC адреса ESP
String id() {
	int mac_len = WL_MAC_ADDR_LENGTH;
	uint8_t mac[mac_len];

	WiFi.softAPmacAddress(mac);

	String mac_id = String(mac[mac_len - 2], HEX) +
					String(mac[mac_len - 1], HEX);

	return mac_id;
}

// Запуск ESP в режиме SoftAP
bool start_ap_mode() {
	IPAddress ap_IP(192, 168, 4, 1);
	IPAddress subnet(255, 255, 255, 0);

	// String network_name = AP_NAME + id();
	String network_name = AP_NAME;

	WiFi.disconnect();
	WiFi.mode(WIFI_AP);
	WiFi.softAPConfig(ap_IP, ap_IP, subnet); // IP, Gateway, Subnet
	WiFi.softAP(network_name.c_str(), AP_PASSWORD.c_str());

	Serial.print("WiFi started in AP mode: ");
	Serial.print(network_name);
	Serial.print("\n");

	return true;
}

bool start_client_mode(){
	wifiMulti.addAP(CLIENT_SSID, CLIENT_PASS);
	Serial.println("Start client mode");
	while(wifiMulti.run() != WL_CONNECTED){
		delay(10);
	}
	return true;
}

void init_Wifi(bool AP_mode){
	if (AP_mode){
		start_ap_mode();
		ip = WiFi.softAPIP().toString();
	}else{
		start_client_mode();
		ip = WiFi.localIP().toString();
	}
	Serial.print("IP adress:");
	Serial.println(ip);
}