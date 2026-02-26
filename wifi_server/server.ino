#include <ESP8266WebServer.h>

static ESP8266WebServer server(80);

static void handle_root() {
  char* page_code = "<form action='led' method='POST'>"
                    "<input type='submit' value='Switch LED'>"
                    "</form>";

  server.send(200, "text/html", page_code);
}

static void handle_led() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  server.sendHeader("Location", "/");
  server.send(303);
}

static void handle_not_found() {
  server.send(404, "text/html", "404 check url");
}

void server_init() {
  server.on("/", HTTP_GET, handle_root);
  server.on("/led", HTTP_POST, handle_led);
  server.onNotFound(handle_not_found);
  server.begin();

  Serial.print("Server is up on port 80\n");
}

void server_handle() {
  server.handleClient();
}
