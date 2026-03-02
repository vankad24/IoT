
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



void setup(){
	pinMode(DIR_LEFT, OUTPUT);
	pinMode(DIR_RIGHT, OUTPUT);
	pinMode(SPEED_LEFT, OUTPUT);
	pinMode(SPEED_RIGHT, OUTPUT);

	
	//move(FORWARD_LEFT, 100, FORWARD_RIGHT, 255);
	turn_left(150);
	delay(2000);

	rotate_right(150);
	delay(2000);
	stop();
}

void loop(){


}
