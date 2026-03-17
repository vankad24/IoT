
#include <SoftwareSerial.h>

#define DIR_LEFT 4
#define DIR_RIGHT 7
#define SPEED_RIGHT 6
#define SPEED_LEFT 5

#define FORWARD_LEFT LOW
#define FORWARD_RIGHT LOW

void move(bool left_dir, int left_speed, bool right_dir, int right_speed){
	digitalWrite(DIR_LEFT, left_dir);
	digitalWrite(DIR_RIGHT, right_dir);	
	analogWrite(SPEED_RIGHT, right_speed);
	analogWrite(SPEED_LEFT, left_speed);
}

void backward(int left_speed, int right_speed){
	move(!FORWARD_LEFT, left_speed, !FORWARD_RIGHT, right_speed);	
}

void backward(int speed){
	backward(speed, speed);
}

void forward(int left_speed, int right_speed){
	move(FORWARD_LEFT, left_speed, FORWARD_RIGHT, right_speed);	
}

void forward(int speed){
	forward(speed, speed);
}

void stop(){
	forward(0);
}

//steepness - насколько круто поворачивать где 0 - длинный поворот, а 255 - крутой.
void turn_left(int steepness){
	int speed = 255;
	forward(255-steepness, speed);
}

void turn_right(int steepness){
	int speed = 255;
	forward(speed, 255-steepness);
}

//вращение на месте
void rotate_left(int speed){
	move(!FORWARD_LEFT, speed, FORWARD_RIGHT, speed);
}

void rotate_right(int speed){
	move(FORWARD_LEFT, speed, !FORWARD_RIGHT, speed);
}



SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
	pinMode(DIR_LEFT, OUTPUT);
	pinMode(DIR_RIGHT, OUTPUT);
	pinMode(SPEED_LEFT, OUTPUT);
	pinMode(SPEED_RIGHT, OUTPUT);
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
}

void loop() { // run over and over
  if (mySerial.available()) {
    // Serial.write(mySerial.read());
    char c = mySerial.read();
    if (c=='F'){
    	forward(255);
    }else if (c=='S'){
    	stop();
    }else if (c=='B'){
    	backward(255);
    }else if (c=='L'){
    	rotate_left(255);
    }else if (c=='R'){
    	rotate_right(255);
    }

  }
  // if (Serial.available()) {
    // mySerial.write(Serial.read());
  // }
}