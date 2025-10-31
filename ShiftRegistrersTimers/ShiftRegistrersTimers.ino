// Библиотеки для регистров
#include <avr/io.h>
#include <avr/interrupt.h>


// --- ФЛАГ ТИПА ИНДИКАТОРА ---
// Раскомментируйте одну из строк:
// #define COMMON_ANODE  // Для индикаторов с общим анодом (CA)
#define COMMON_CATHODE // Для индикаторов с общим катодом (CC)

// Константы пинов для PORTD (цифровые пины 0-7)
const uint8_t PIN_3_MASK = (1 << PD3); //(Pin 3)
const uint8_t PIN_5_MASK = (1 << PD5); //(Pin 5)
const uint8_t PIN_7_MASK = (1 << PD7); //(Pin 7)

// Маска для всех пинов вместе
const uint8_t ALL_PINS_MASK = PIN_3_MASK | PIN_5_MASK | PIN_7_MASK;

// Пины для управления 74HC595
const uint8_t LATCH_PIN_MASK  = PIN_5_MASK; //ST_CP
const uint8_t CLOCK_PIN_MASK = PIN_3_MASK; //SH_CP
const uint8_t DATA_PIN_MASK  = PIN_7_MASK; //DS


// Переменные и константы
volatile uint32_t current_millis = 0;
volatile uint32_t next_call_millis = 0;
const uint16_t FUNC_CALL_INTERVAL_MS = 1000;

volatile uint16_t firstDigit = 0;
volatile uint16_t secondDigit = 0;
volatile uint32_t start_millis = 0;
volatile int start_value = -1;
volatile int number_to_show = -1;
volatile bool need_to_show = true;


// Биты для отображения на семисегментном индикаторе
// Данные для общего катода (единица включает сегмент)
#ifdef COMMON_CATHODE
//  AB.CDEGF
uint8_t digits[10] = {
  0b11011101, // 0
  0b01010000, // 1
  0b11001110, // 2
  0b11011010, // 3
  0b01010011, // 4
  0b10011011, // 5
  0b10011111, // 6
  0b11010000, // 7
  0b11011111, // 8
  0b11011011  // 9
};
uint8_t empty_segments = 0b00000000;
#endif

// Данные для общего анода (ноль включает сегмент)
#ifdef COMMON_ANODE
//  AB.CDEGF (Инверсия COMMON_CATHODE)
uint8_t digits[10] = {
  0b00100010, // 0
  0b10101111, // 1
  0b00110001, // 2
  0b00100101, // 3
  0b10101100, // 4
  0b01100100, // 5
  0b01100000, // 6
  0b00101111, // 7
  0b00100000, // 8
  0b00100100  // 9
};
uint8_t empty_segments = 0b11111111;
#endif

//Функции
void setPinModeOutput(uint8_t mask) {
  DDRD |= mask;
}

void setPinModeInput(uint8_t mask) {
  DDRD &= ~mask;
}

void pinHigh(uint8_t mask) {
  PORTD |= mask;
}

void pinLow(uint8_t mask) {
  PORTD &= ~mask;
}


void shiftOutByte(uint8_t dataPinMask, uint8_t clockPinMask, uint8_t val) {
  // Инициализация: Clock Low
  pinLow(clockPinMask);
  for (int i = 0; i < 8; i++) {
    // Установка бита данных
    if (val & (1 << i)) {
      pinHigh(dataPinMask);
    } else {
      pinLow(dataPinMask);
    }

    pinHigh(clockPinMask);
    pinLow(clockPinMask);
  }
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
}


// Прерывание Timer1 каждые 1 мс
ISR(TIMER1_COMPA_vect) {
  current_millis++;
  if (current_millis>=next_call_millis){
    next_call_millis += FUNC_CALL_INTERVAL_MS;
    showTimeOnClocks();
  }
}


void setup() {
  setPinModeOutput(ALL_PINS_MASK);
  pinLow(ALL_PINS_MASK);
    
  setupInterrupts();

  Serial.begin(9600);
}

void showByte(uint8_t b){
  pinLow(LATCH_PIN_MASK);
  shiftOutByte(DATA_PIN_MASK, CLOCK_PIN_MASK, b);
  pinHigh(LATCH_PIN_MASK);
}

void showDigit(int index) {
  showByte(digits[index]);
}

void showNumber(int num) {
  int d = num%100/10;
  if (d==0) showByte(empty_segments);
  else showDigit(d);
  showDigit(num%10);
}

void showTimeOnClocks() {
  if (number_to_show!=-1) {
    if (start_value<0){
      start_millis = current_millis;
      start_value = number_to_show;
    }
    if (need_to_show){
      showNumber(number_to_show);
      need_to_show = false;
      firstDigit = 0;
      secondDigit = 0;
    }else{
      int seconds = ((current_millis-start_millis)/1000+start_value)%60;
      showNumber(seconds);
    }
  }
}

void loop() {
  if (Serial.available()){
    char inChar = (char)Serial.read();
    if (inChar >= '0' && inChar <= '9') {
      int digit = (int)(inChar - '0');
      secondDigit = firstDigit;
      firstDigit = digit;
      need_to_show = true;
    }
    // Безопасная запсись 4 байтной переменной
    cli();
    number_to_show = secondDigit*10 + firstDigit;
    sei();
  }
}
