#include <PubSubClient.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

extern WiFiClient wifiClient;
PubSubClient mqtt_client(wifiClient);
extern String id();

const int mqtt_port = 1883;
char mqtt_broker[] = "broker.emqx.io";


void callback(char *topic, byte *payload, unsigned int len){
	Serial.print("There is message: "); 
	Serial.println(topic);
	Serial.print("message: ");
	for (int i = 0; i < len; i++){
		Serial.print((char)payload[i]);
	}
	Serial.println("");
}

bool init_mqtt(){
	mqtt_client.setServer(mqtt_broker, mqtt_port);
	mqtt_client.setCallback(callback);
	while(!mqtt_client.connected()){
		String client_id = "esp8266_"+id();
		if (mqtt_client.connect(client_id.c_str())){
			Serial.println("MQTT client connected with id "+client_id);
		}else{
			Serial.println("MQTT client does not connect");
			delay(500);
		}
	}
	return true;
}