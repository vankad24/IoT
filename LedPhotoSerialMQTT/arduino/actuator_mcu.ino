const int led_pin = LED_BUILTIN;
bool blinking = false;
unsigned long blinkInterval = 500;
unsigned long lastBlink = 0;
bool ledState = false;

void setup() {
  pinMode(led_pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'u') {
      blinking = false;
      digitalWrite(led_pin, HIGH);
      Serial.println("LED_ON");
    } else if (cmd == 'd') {
      blinking = false;
      digitalWrite(led_pin, LOW);
      Serial.println("LED_OFF");
    } else if (cmd == 'b') {
      blinking = true;
      lastBlink = millis();
      Serial.println("LED_BLINK");
    }
  }

  if (blinking) {
    unsigned long now = millis();
    if (now - lastBlink >= blinkInterval) {
      ledState = !ledState;
      digitalWrite(led_pin, ledState ? HIGH : LOW);
      lastBlink = now;
    }
  }
}