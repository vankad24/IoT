// #include <Arduino.h> // Если используете Arduino, иначе <stdint.h>
#include <stdint.h> // Если используете Arduino, иначе <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

//generated arrays
uint8_t sym_to_code_key[] = {33,44,46,48,49,50,51,52,53,54,55,56,57,63,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,};

uint8_t sym_to_code_value[] = {
    0b1101011, //!
    0b1110011, //,
    0b1010101, //.
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
    0b1001100, //?
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

uint8_t sym_to_segment_value[] = {
//    ABCDEFG.
    0b01100010, //!
    0b00001000, //,
    0b00000001, //.
    0b11111100, //0
    0b01100000, //1
    0b11011010, //2
    0b11110010, //3
    0b01100110, //4
    0b10110110, //5
    0b10111110, //6
    0b11100000, //7
    0b11111110, //8
    0b11110110, //9
    0b11001010, //?
    0b11101110, //A
    0b00111110, //B
    0b10011100, //C
    0b01111010, //D
    0b10011110, //E
    0b10001110, //F
    0b10111100, //G
    0b01101110, //H
    0b01100001, //I
    0b01111000, //J
    0b10101111, //K
    0b00011100, //L
    0b11101101, //M
    0b00101010, //N
    0b11111101, //O
    0b11001110, //P
    0b11100110, //Q
    0b00001010, //R
    0b10110111, //S
    0b00011111, //T
    0b01111100, //U
    0b00111000, //V
    0b01010101, //W
    0b01101111, //X
    0b01110110, //Y
    0b11011011, //Z
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
    0b1001100, //?
    0b1010101, //.
    0b1101011, //!
    0b1110011, //,
};

char code_to_sym_value[] = {'E','T','I','A','N','M','S','U','R','W','D','K','G','O','H','V','F','L','P','J','B','X','C','Y','Z','Q','5','4','3','2','1','6','7','8','9','0','?','.','!',',',};

const uint8_t ALPHABET_LEN = 40;

// ----------- enums -----------

enum TRANSFER_STATES {
    WAIT_LETTER,
    BIT,
    PAUSE,
};

enum MODE_STATES {
  ROW,
  NORMAL,
  SER,
  WAIT
};

// ----------- classes -----------
class Buffer {
private:
    volatile uint8_t head;
    volatile uint8_t tail;
    const uint8_t size; // N теперь здесь
    char* buffer;

public:
    // Конструктор: выделяем память динамически
    Buffer(uint8_t n) : head(0), tail(0), size(n) {
        buffer = new char[size];
    }

    // Деструктор: обязательно освобождаем память
    ~Buffer() {
        delete[] buffer;
    }

    // Запрещаем копирование (чтобы не удалить один массив дважды)
    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

    // ===== сколько элементов можно ПРОЧИТАТЬ =====
    int available() const volatile{
        uint8_t h, t;
        uint8_t sreg = SREG;
        cli();
        h = head;
        t = tail;
        SREG = sreg;

        if (h >= t)
            return h - t;
        else
            return size - t + h;
    }

    // ===== сколько элементов можно ЗАПИСАТЬ =====
    int writeAvailable() const volatile{
        uint8_t h, t;
        uint8_t sreg = SREG;
        cli();
        h = head;
        t = tail;
        SREG = sreg;

        if (h >= t)
            return (size - 1) - (h - t);
        else
            return (t - h - 1);
    }

    // ===== чтение =====
    bool read(char &out) volatile{
        uint8_t sreg = SREG;
        cli();

        if (head == tail) {
            SREG = sreg;
            return false;
        }

        out = buffer[tail];
        tail = (tail + 1) % size;

        SREG = sreg;
        return true;
    }

    // ===== запись =====
    bool write(const char value) volatile{
        uint8_t sreg = SREG;
        cli();

        uint8_t next = (head + 1) % size;
        if (next == tail) {
            SREG = sreg;
            return false;
        }

        buffer[head] = value;
        head = next;

        SREG = sreg;
        return true;
    }
};
// ----------- pins -----------

const uint8_t SENSOR_PIN = 2;
const uint8_t MORSE_CODE_RX_PIN = 3; // interrupt pin
const uint8_t MORSE_CODE_TX_PIN = 4;

// Пины для управления 74HC595
const uint8_t CLOCK_PIN = 6; //SH_CP
const uint8_t LATCH_PIN = 7; //ST_CP
const uint8_t DATA_PIN = 8; //DS


/*
// ESP8266 pins
const uint8_t MORSE_CODE_RX_PIN = 0; //D3 // interrupt pin
const uint8_t MORSE_CODE_TX_PIN = 2; //D4
const uint8_t SENSOR_PIN = 14; //D5

// Пины для управления 74HC595
const uint8_t CLOCK_PIN = 12;//D6 //SH_CP
const uint8_t LATCH_PIN = 13;//D7 //ST_CP
const uint8_t DATA_PIN = 15;//D8 //DS
*/


// ----------- Настройки буфера -----------
#define TX_BUFFER_SIZE 64
#define RX_BUFFER_SIZE 64

// ----------- durations and intervals -----------
const uint32_t CHAR_DISPLAY_TIME_MS = 500;
// TU - Time Unit (ms)
const uint8_t TIME_UNIT_MS = 50;
const uint8_t DEBOUNCE_MAX_LIMIT_MS = 10;
//HIGH intervals
const uint8_t DOT_TU = 1;
const uint8_t DASH_TU = 3;
//LOW intervals
const uint8_t ELEMENT_PAUSE_TU = 1;
const uint8_t LETTER_PAUSE_TU = 3;
const uint8_t WORD_PAUSE_TU = 7;
// TIMEOUT HIGH or LOW
const uint8_t TIMEOUT_TU = 10;

// ----------- calculated interval limits -----------
const float DOT_LEFT_LIMIT = DOT_TU/2.0f;
const float DOT_RIGHT_LIMIT = DOT_TU + (DASH_TU-DOT_TU)/2.0f;
const float DASH_RIGHT_LIMIT = (float)TIMEOUT_TU;

const float ELEMENT_PAUSE_LEFT_LIMIT = ELEMENT_PAUSE_TU/2.0f;
const float ELEMENT_PAUSE_RIGHT_LIMIT = ELEMENT_PAUSE_TU + (LETTER_PAUSE_TU-ELEMENT_PAUSE_TU)/2.0f;
const float LETTER_PAUSE_RIGHT_LIMIT = LETTER_PAUSE_TU + (WORD_PAUSE_TU-LETTER_PAUSE_TU)/2.0f;
const float WORD_PAUSE_RIGHT_LIMIT = (float)TIMEOUT_TU;

// ----------- variables -----------
volatile uint64_t current_millis = 0;
volatile uint64_t last_change = 0;
volatile uint64_t last_stable_change_time = 0;
volatile uint64_t rx_timeout_time = 0;
volatile uint64_t tx_timeout_time = 0;
volatile uint64_t no_debounce_accept_time = 0;
volatile uint64_t next_tx_change_time = 0;
volatile uint8_t rx_code = 1;
volatile uint8_t tx_code = 1;
volatile uint8_t last_pause_tu = 1;
volatile int8_t shift_tx_code = -1;
volatile bool rx_signal_value = true;
volatile bool tx_signal_value = true;
volatile bool is_reading_status = false;
volatile bool debounce_checking = false;
volatile bool saved_signal_value = false;
volatile TRANSFER_STATES tx_next_state = WAIT_LETTER;
volatile Buffer rx_buffer(RX_BUFFER_SIZE);
volatile Buffer tx_buffer(TX_BUFFER_SIZE);

// ----------- loop variables -----------
volatile MODE_STATES mode = SER;
volatile MODE_STATES prev_mode = SER;
uint64_t next_char_display_time = 0;
char cur_char = 0;

volatile uint64_t last_debug = 0;


// ----------- Функции для получения данных из "словарей" -----------

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

uint8_t symToSegment(char c){
    int index = binarySearch(sym_to_code_key, ALPHABET_LEN, (uint8_t)c);
    return index>=0 ? sym_to_segment_value[index] : 0;
}

// ----------- Функции чтения морзе -----------

void addBitToCode(bool is_one){
    rx_code<<=1;
    rx_code|=is_one;
}

void resetCode(){
    rx_code=1;
}

void saveSym(char c){
    rx_buffer.write(c);
    resetCode();
}

void readSignal(){
    float delta_tu = (float)(last_change - last_stable_change_time) / TIME_UNIT_MS;


    if (rx_signal_value){
        //from HIGH to LOW
        
        if (delta_tu < DOT_LEFT_LIMIT){
            // дребезг
        }else if (DOT_LEFT_LIMIT <= delta_tu && delta_tu < DOT_RIGHT_LIMIT){
            //DOT_TU action
            addBitToCode(0);
        }else if (DOT_RIGHT_LIMIT <= delta_tu && delta_tu < DASH_RIGHT_LIMIT){
            //DASH_TU action
            addBitToCode(1);
        }else {
            // TIMEOUT
        }

    }else{
        //from LOW to HIGH
        if (delta_tu < ELEMENT_PAUSE_LEFT_LIMIT) {
            // дребезг
        } else if (ELEMENT_PAUSE_LEFT_LIMIT <= delta_tu && delta_tu < ELEMENT_PAUSE_RIGHT_LIMIT) {
            // ELEMENT_PAUSE action
        } else if (ELEMENT_PAUSE_RIGHT_LIMIT <= delta_tu && delta_tu < LETTER_PAUSE_RIGHT_LIMIT) {
            // LETTER_PAUSE action
            saveSym(codeToSym(rx_code));
        } else if (LETTER_PAUSE_RIGHT_LIMIT <= delta_tu && delta_tu < WORD_PAUSE_RIGHT_LIMIT) {
            // WORD_PAUSE action
            saveSym(codeToSym(rx_code));
            saveSym(' ');
        } else {
            // TIMEOUT
        }
        
    }

}

// ----------- Функции отправки морзе -----------

void sendSignal() {
    
    switch (tx_next_state) {
        case WAIT_LETTER:
            // В простое проверяем буфер
            if (tx_buffer.available()) {
                char c;
                tx_buffer.read(c);

                if (c == ' ') {
                    // Пробел — это пауза между словами
                    // Т.к. мы уже делали одну из пауз, нужно ждать меньше
                    changeTxSignal(WORD_PAUSE_TU - last_pause_tu, false);

                    tx_next_state = WAIT_LETTER; // После паузы снова ищем данные
                } else {
                    // Переводим символ в код
                    tx_code = symToCode(c);
                    if (tx_code == 0) {
                        
                        return; // Пропуск неизвестных символов
                    }

                    // Ищем старшую единицу
                    int8_t shift = 7;
                    while (shift >= 0 && !((tx_code >> shift) & 1)) {
                        shift--;
                    }
                    
                    if (shift < 0) return; // Ошибка кодировки
                    
                    shift_tx_code = shift - 1; // Начинаем с бита после маркера

                    tx_next_state = BIT;
                    
                    if (tx_timeout_time<current_millis){
                        // timeout уже был, начинаем новую передачу
                        // делаем стартовую паузу
                        changeTxSignal(ELEMENT_PAUSE_TU, false);
                    }else sendSignal();// Сразу переходим к отправке первого бита буквы (Рекурсивный вызов без ожидания)
                }
            } else {
                // Буфер пуст, ожидаем в HIGH
                digitalWrite(MORSE_CODE_TX_PIN, HIGH);
                tx_signal_value = true;
                
            }
            break;

        case BIT:
            {
                // Читаем текущий бит (0 - точка, 1 - тире)
                bool is_dash = (tx_code >> shift_tx_code) & 1;
                changeTxSignal(is_dash ? DASH_TU : DOT_TU, true);

                // Следующее состояние — всегда пауза между элементами
                tx_next_state = PAUSE;
            }
            break;

        case PAUSE:
            // Определяем тип паузы
            if (shift_tx_code > 0) {
                // ELEMENT_PAUSE - пауза после точки или тире
                shift_tx_code--;
                changeTxSignal(ELEMENT_PAUSE_TU, false);
                last_pause_tu = ELEMENT_PAUSE_TU;
                // ещё остались биты в букве, продолжаем
                tx_next_state = BIT;
            } else {
                // Биты кончились, это конец буквы.
                changeTxSignal(LETTER_PAUSE_TU, false);
                last_pause_tu = LETTER_PAUSE_TU;
                // После паузы буквы идем за новой буквой
                tx_next_state = WAIT_LETTER;
            }
            break;
        default:
            tx_next_state = WAIT_LETTER;
            break;
    }
}

void changeTxSignal(uint8_t duration, bool signal_value){
    tx_signal_value = signal_value;
    digitalWrite(MORSE_CODE_TX_PIN, signal_value);
    next_tx_change_time = current_millis + (uint64_t)duration * TIME_UNIT_MS;
}

// ----------- Вызовы прерываний -----------

void pinInterrupt() {
    // Просто фиксируем, что что-то изменилось
    // и запоминаем время последнего
    last_change = current_millis;
}

void timerInterrupt() {
    // 1. Обработка дебаунса и смены состояний
    // Если сигнал стабилен больше DEBOUNCE_MAX_LIMIT_MS и мы еще не обработали это
    bool actual_pin_state = digitalRead(MORSE_CODE_RX_PIN);
    
    // Если состояние пина не совпадает с логическим rx_signal_value и дребезг прошел
    if (actual_pin_state != rx_signal_value && (current_millis - last_change) >= DEBOUNCE_MAX_LIMIT_MS) {
        
        // ВАЖНО: Мы замеряем длительность ПРЕДЫДУЩЕГО состояния
        // Которое длилось с момента ПРОШЛОЙ стабильной смены до СЕЙЧАС (точнее до last_change)
        
        // Обрабатываем завершенное состояние (rx_signal_value — это то, что БЫЛО)
        
        readSignal();
        
        // Обновляем состояние
        rx_signal_value = actual_pin_state;
        last_stable_change_time = last_change;
        
        // Сбрасываем таймаут
        rx_timeout_time = current_millis + (uint32_t)TIMEOUT_TU * TIME_UNIT_MS;
        is_reading_status = true;
    }

    // 2. Обработка таймаута (если долго нет изменений)
    if (is_reading_status && current_millis >= rx_timeout_time) {
        is_reading_status = false;
        if (rx_code > 1) { // Если в коде что-то есть
            saveSym(codeToSym(rx_code));
        }
    }

    // отправка
    if (current_millis >= next_tx_change_time) sendSignal();

    if (last_tx_state!=tx_signal_value){
        last_tx_state = tx_signal_value;
        tx_timeout_time = current_millis + (uint64_t) TIMEOUT_TU * TIME_UNIT_MS;
    }
}


void sensorInterrupt(){
    if (mode==ROW || mode == NORMAL){
        bool level = !digitalRead(SENSOR_PIN);
        digitalWrite(MORSE_CODE_TX_PIN, level);
        Serial.print("sensor ");
        Serial.println(level);
    }
}

// -------- segments --------

void sendByteToSegments(uint8_t value) {
  digitalWrite(LATCH_PIN, LOW); // Начинаем передачу данных
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, value); // Передаём байт
  digitalWrite(LATCH_PIN, HIGH); // Завершаем передачу
}

void showSymbol(char c){
    sendByteToSegments(symToSegment(c));
}

// ----------- Функции взаимодействия с буферами -----------

int morseAvailable() {
    return rx_buffer.available();
}

char morseRead() {
    char c;
    if (!rx_buffer.read(c))return 0;
    return c;
}

int morseWriteAvailable(){
    return tx_buffer.writeAvailable();
}

bool morseWrite(char c) {
  return tx_buffer.write(c);
}

// ----------- Настройки прерываний -----------

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
  attachInterrupt(digitalPinToInterrupt(MORSE_CODE_RX_PIN), pinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorInterrupt, CHANGE);
}


// Прерывание Timer1 каждые 1 мс
ISR(TIMER1_COMPA_vect) {
    current_millis++; //работает быстрее, чем millis()
    timerInterrupt();
}


// ----------- setup/loop -----------

void setup(){
    Serial.begin(9600);
    // Настройка пинов с подтягивающим резистором для избегания помех
    pinMode(MORSE_CODE_RX_PIN, INPUT_PULLUP);
    pinMode(SENSOR_PIN, INPUT_PULLUP);
    
    // Пин передачи
    pinMode(MORSE_CODE_TX_PIN, OUTPUT);

    // Пины для управления 74HC595
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);

    setupInterrupts();


    /*rx_buffer.write('A');
    rx_buffer.write('B');
    rx_buffer.write('C');

      char c;
      while (morseAvailable()) {
        Serial.print("Available read ");
        Serial.println(morseRead());
      }*/

    Serial.println("Started");
}

void loop() {
    // передача
    if (cur_char == 0){
        if (Serial.available()) cur_char = Serial.read();
    }else{
        if (cur_char == '/') {
            prev_mode = mode;
            mode = WAIT;
            cur_char = 0;

            Serial.println("Available modes:");
            Serial.println("'r' - row");
            Serial.println("'n' - normal");
            Serial.println("'s' - serial");
        }
        else if (mode == WAIT) {
            switch (cur_char) {
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
            cur_char = 0;
        }else{
          switch (mode) {
            case ROW:
              // row logic
              break;
            case NORMAL:
              // normal logic
              break;
            case SER:
              // serial logic
                if (morseWriteAvailable()){
                    morseWrite(cur_char);
                    cur_char = 0;
                }
              break;
            default:
              break;
            }
        }
    }

    // чтение
    uint64_t cur_millis = millis();
    if (next_char_display_time <= cur_millis && morseAvailable()){
        char sym = morseRead();
        showSymbol(sym);
        // Serial.print("Recieved char: ");
        Serial.print(sym);
        next_char_display_time = cur_millis + CHAR_DISPLAY_TIME_MS;
    }
}




