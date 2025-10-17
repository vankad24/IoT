const int LDR_PIN = A0;
unsigned long streamInterval = 1000;
bool streaming = false;
unsigned long lastStream = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'p') {
      int val = analogRead(LDR_PIN);
      Serial.print("SENSOR_VALUE:");
      Serial.println(val);
    } else if (cmd == 's') {
      streaming = true;
      Serial.println("STREAM_STARTED");
    } else if (cmd == 'x') {
      streaming = false;
      Serial.println("STREAM_STOPPED");
    }
  }

  if (streaming) {
    unsigned long now = millis();
    if (now - lastStream >= streamInterval) {
      int val = analogRead(LDR_PIN);
      Serial.print("SENSOR_VALUE:");
      Serial.println(val);
      lastStream = now;
    }
  }
}