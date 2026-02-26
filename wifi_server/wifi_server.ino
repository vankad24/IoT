#include "wifi.h"
#include "server.h"
#include "mqtt.h"

//ap - access point

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);

  Serial.begin(9600);
  Serial.print("\f\rSystem startup\n");

  // start_ap_mode();
  init_Wifi(false);
  server_init();
  init_mqtt();
  String topic = "esp8266_242424";
  String publish_topic = "esp8266_24242424";
  mqtt_client.subscribe(topic.c_str());
  mqtt_client.publish(publish_topic.c_str(), "Hello, world!");
  Serial.println("Publish on topic " + publish_topic);
}

void loop() {
  server_handle();
  mqtt_client.loop();
  delay(20);
}
