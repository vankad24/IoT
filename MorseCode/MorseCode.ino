// #include <Arduino.h> // Если используете Arduino, иначе <stdint.h>
#include <stdint.h> // Если используете Arduino, иначе <stdint.h>

//generated arrays
uint8_t sym_to_code_key[] = {48,49,50,51,52,53,54,55,56,57,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,};

uint8_t sym_to_code_value[] = {
    0b111111, //0
    0b101111, //1
    0b100111, //2
    0b100011, //3
    0b100001, //4
    0b100000, //5
    0b110000, //6
    0b111000, //7
    0b111100, //8
    0b111110, //9
    0b101, //A
    0b11000, //B
    0b11010, //C
    0b1100, //D
    0b10, //E
    0b10010, //F
    0b1110, //G
    0b10000, //H
    0b100, //I
    0b10111, //J
    0b1101, //K
    0b10100, //L
    0b111, //M
    0b110, //N
    0b1111, //O
    0b10110, //P
    0b11101, //Q
    0b1010, //R
    0b1000, //S
    0b11, //T
    0b1001, //U
    0b10001, //V
    0b1011, //W
    0b11001, //X
    0b11011, //Y
    0b11100, //Z
};

uint8_t code_to_sym_key[] = {
    0b10, //E
    0b11, //T
    0b100, //I
    0b101, //A
    0b110, //N
    0b111, //M
    0b1000, //S
    0b1001, //U
    0b1010, //R
    0b1011, //W
    0b1100, //D
    0b1101, //K
    0b1110, //G
    0b1111, //O
    0b10000, //H
    0b10001, //V
    0b10010, //F
    0b10100, //L
    0b10110, //P
    0b10111, //J
    0b11000, //B
    0b11001, //X
    0b11010, //C
    0b11011, //Y
    0b11100, //Z
    0b11101, //Q
    0b100000, //5
    0b100001, //4
    0b100011, //3
    0b100111, //2
    0b101111, //1
    0b110000, //6
    0b111000, //7
    0b111100, //8
    0b111110, //9
    0b111111, //0
};

char code_to_sym_value[] = {'E','T','I','A','N','M','S','U','R','W','D','K','G','O','H','V','F','L','P','J','B','X','C','Y','Z','Q','5','4','3','2','1','6','7','8','9','0',};

const uint8_t ALPHABET_LEN = 36;

enum DATA_STATES {
    IDLE,
    WAIT_EDGE,
    MARK,
    SPACE,
    END_SYMBOL,
    END_MESSAGE
};

enum MODE_STATES {
  ROW,
  NORMAL,
  SER,
  WAIT
};

// ----------- pins -----------
const uint8_t MORSE_CODE_PIN = 3; // interrupt pin
const uint8_t MORSE_CODE_TX_PIN = 4;
const uint8_t SENSOR_PIN = 5;

// ----------- durations and intervals -----------
// TU - Time Unit (ms)
const uint8_t TIME_UNIT_MS = 50;
//HIGH intervals
const uint8_t DOT_TU = 1;
const uint8_t DASH_TU = 3;
//LOW intervals
const uint8_t ELEMENT_PAUSE_TU = 1;
const uint8_t LETTER_PAUSE_TU = 3;
const uint8_t WORD_PAUSE_TU = 7;
// TIMEOUT HIGH or LOW
const uint8_t TIMEOUT_TU = 10;


const float DOT_LEFT_LIMIT = DOT_TU/2.0f;
const float DOT_RIGHT_LIMIT = DOT_TU + (DASH_TU-DOT_TU)/2.0f;
const float DASH_RIGHT_LIMIT = (float)TIMEOUT_TU;

const float ELEMENT_PAUSE_LEFT_LIMIT = ELEMENT_PAUSE_TU/2.0f;
const float ELEMENT_PAUSE_RIGHT_LIMIT = ELEMENT_PAUSE_TU + (LETTER_PAUSE_TU-ELEMENT_PAUSE_TU)/2.0f;
const float LETTER_PAUSE_RIGHT_LIMIT = LETTER_PAUSE_TU + (WORD_PAUSE_TU-LETTER_PAUSE_TU)/2.0f;
const float WORD_PAUSE_RIGHT_LIMIT = (float)TIMEOUT_TU;





volatile uint64_t current_millis = 0;
volatile uint64_t last_change = 0;
volatile uint8_t current_code = 1;
volatile bool signal_state = true;


MODE_STATES mode = SER;
MODE_STATES prev_mode = SER;


int binarySearch(uint8_t* ptr, int len, uint8_t target) {
    int left = 0;
    int right = len - 1;

    while (left <= right) {
        int mid = left + (right - left) / 2;

        if (*(ptr + mid) == target) {
            return mid;
        }

        if (*(ptr + mid) < target) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    return -1;
}


uint8_t symToCode(char c){
	int index = binarySearch(sym_to_code_key, ALPHABET_LEN, (uint8_t)c);
	return index>=0 ? sym_to_code_value[index] : 0;
}


char codeToSym(uint8_t code){
	int index = binarySearch(code_to_sym_key, ALPHABET_LEN, code);
	return index>=0 ? code_to_sym_value[index] : '?';
}

void addBitToCode(bool is_one){
	current_code<<=1;
	current_code|=is_one;
}

void sendCode(uint8_t code){
	//отправляем код слева направо
	// todo неблокирующе
	int shift = 7;
	//находим первую единицу
	while (! (code>>shift)){
		shift --;
	}
	//отправляем биты
	while (shift>0){
		shift--;
		sendBit((code>>shift)&1);
	}
}

void sendBit(bool is_one){

}

bool isCloserToLess(uint8_t left, uint8_t right, float delta_tu){
	// ближе ли к меньшему числу или большему из двух: left и right
	// return (delta_tu-left)<(right-delta_tu); до преобразований.
	// После:
	return delta_tu+delta_tu<left+right;
}

void pinInterrupt() {
	//reading
	signal_state = !signal_state;
	float delta_tu = (float)(current_millis - last_change)/TIME_UNIT_MS;

	if (signal_state){
		//from LOW to HIGH

		
		if (delta_tu < ELEMENT_PAUSE_LEFT_LIMIT) {
		    // дребезг
		} else if (ELEMENT_PAUSE_LEFT_LIMIT <= delta_tu && delta_tu < ELEMENT_PAUSE_RIGHT_LIMIT) {
		    // ELEMENT_PAUSE action
		} else if (ELEMENT_PAUSE_RIGHT_LIMIT <= delta_tu && delta_tu < LETTER_PAUSE_RIGHT_LIMIT) {
		    // LETTER_PAUSE action
		} else if (LETTER_PAUSE_RIGHT_LIMIT <= delta_tu && delta_tu < WORD_PAUSE_RIGHT_LIMIT) {
		    // WORD_PAUSE action
		} else {
		    // TIMEOUT
		}


	}else{
		//from HIGH to LOW
		
		if (delta_tu < DOT_LEFT_LIMIT){
			// дребезг
		}else if (DOT_LEFT_LIMIT >= delta_tu && delta_tu < DOT_RIGHT_LIMIT){
			//DOT_TU action
		}else if (DOT_RIGHT_LIMIT >= delta_tu && delta_tu < DASH_RIGHT_LIMIT){
			//DASH_TU action
		}else {
		    // TIMEOUT
		}
		
	}

}

void timerInterrupt() {



}



void setupInterrupts() {
  // Настройка Timer1 (1 мс тик)
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  // CTC режим: TOP = OCR1A
  TCCR1B |= (1 << WGM12);
  // OCR1A под 1 мс (16e6 / 64 / 1000 = 250)
  OCR1A = 249;
  // предделитель 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // разрешить прерывание по совпадению
  TIMSK1 |= (1 << OCIE1A);
  sei();

  // Прерывания по пину
  attachInterrupt(digitalPinToInterrupt(MORSE_CODE_PIN), pinInterrupt, CHANGE);
}


// Прерывание Timer1 каждые 1 мс
ISR(TIMER1_COMPA_vect) {
	current_millis++; //работает быстрее, чем millis()
	timerInterrupt();
}

void setup(){
	Serial.begin(9600);
	// Настройка пинов с подтягивающим резистором
    pinMode(MORSE_CODE_PIN, INPUT_PULLUP);
    pinMode(SENSOR_PIN, INPUT_PULLUP);
    setupInterrupts();

	uint8_t code = symToCode('B');
	Serial.println(code);
	Serial.println(codeToSym(code));
	
	addBitToCode(true);
	addBitToCode(true);
	addBitToCode(false);

	Serial.println(codeToSym(current_code));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (mode != WAIT) {
      if (c == '/') {
        prev_mode = mode;
        mode = WAIT;

        Serial.println("Available modes:");
        Serial.println("'r' - row");
        Serial.println("'n' - normal");
        Serial.println("'s' - serial");
      }
    } else { // WAIT
		switch (c) {
        case 'r':
          mode = ROW;
          Serial.println("Mode changed: ROW");
          break;
        case 'n':
          mode = NORMAL;
          Serial.println("Mode changed: NORMAL");
          break;
        case 's':
          mode = SER;
          Serial.println("Mode changed: SERIAL");
          break;
        default:
          mode = prev_mode;
          Serial.println("Unknown mode");
          break;
      }
    }
  }

  switch (mode) {
    case ROW:
      // row logic
      break;
    case NORMAL:
      // normal logic
      break;
    case SER:
      // serial logic
      break;
    default:
      break;
  }
}




