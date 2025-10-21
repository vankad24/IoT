#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define UART_TX_PIN PD3
#define UART_RX_PIN PD2
#define BUFFER_SIZE 64

// --- TX Буфер ---
volatile char tx_buffer[BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;
volatile bool tx_active = false;
volatile uint8_t tx_byte_to_send = 0;
volatile uint8_t tx_bit_index = 0; // 0-Старт, 1-8 Данные, 9-Стоп

// --- RX Буфер ---
volatile char rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
volatile uint8_t rx_byte_received = 0;
volatile uint8_t rx_bit_index = 0; // 0-7 Данные
volatile bool rx_active = false;

// --- Общие переменные ---
volatile uint16_t timer_ticks_per_bit = 0;

void uart_set_baudrate(long baud_rate) {
    constexpr uint8_t prescaler = 8;
    timer_ticks_per_bit = (F_CPU / prescaler) / baud_rate;

    // Настройка пинов
    DDRD |= (1 << UART_TX_PIN);
    PORTD |= (1 << UART_TX_PIN); // TX Idle HIGH
    DDRD &= ~(1 << UART_RX_PIN);
    PORTD |= (1 << UART_RX_PIN); // RX Pull-up

    // INT0 на PD2, падающий фронт
    EICRA = (1 << ISC01);
    EIMSK |= (1 << INT0);

    // Timer1, Prescaler 8
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TIMSK1 = 0; // Прерывания пока выключены
}

// --- TX API ---
bool uart_send(char b) {
    uint8_t next_head = (tx_head + 1) % BUFFER_SIZE;
    if (next_head == tx_tail) return false; // Буфер переполнен

    tx_buffer[tx_head] = b;
    tx_head = next_head;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (!tx_active) {
            tx_active = true;
            tx_bit_index = 0;
            OCR1A = TCNT1 + 10; // Первое прерывание почти сразу
            TIMSK1 |= (1 << OCIE1A);
        }
    }
    return true;
}

void uart_send_string(const char* msg) {
    while (*msg) {
        while (!uart_send(*msg)) {}
        msg++;
    }
}

// --- RX API ---
uint8_t uart_available() {
    uint8_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count = (rx_head - rx_tail + BUFFER_SIZE) % BUFFER_SIZE;
    }
    return count;
}

char uart_read() {
    if (rx_head == rx_tail) return -1;

    char b;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        b = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % BUFFER_SIZE;
    }
    return b;
}

// --- TX ISR ---
ISR(TIMER1_COMPA_vect) {
    OCR1A += timer_ticks_per_bit; // Планируем следующее прерывание

    if (tx_bit_index == 0) { // Старт-бит (LOW)
        PORTD &= ~(1 << UART_TX_PIN);
        tx_bit_index++;
    } 
    else if (tx_bit_index >= 1 && tx_bit_index <= 8) { // 8 бит данных (LSB first)
        if (tx_byte_to_send & 1) {
            PORTD |= (1 << UART_TX_PIN);
        } else {
            PORTD &= ~(1 << UART_TX_PIN);
        }
        tx_byte_to_send >>= 1;
        tx_bit_index++;
    } 
    else if (tx_bit_index == 9) { // Стоп-бит (HIGH)
        PORTD |= (1 << UART_TX_PIN);
        tx_bit_index++;
    } 
    else { // Переход к следующему байту или завершение
        if (tx_head != tx_tail) {
            // Берем следующий байт из буфера
            tx_byte_to_send = tx_buffer[tx_tail];
            tx_tail = (tx_tail + 1) % BUFFER_SIZE;
            tx_bit_index = 0; // Начнем со старт-бита в следующем срабатывании
        } else {
            // Буфер пуст. Завершаем передачу.
            tx_active = false;
            TIMSK1 &= ~(1 << OCIE1A);
        }
    }

}

// --- RX Start ISR ---
ISR(INT0_vect) {
    if (rx_active) {
        EIFR |= (1 << INTF0);
        return;
    }

    rx_active = true;
    rx_bit_index = 0;
    rx_byte_received = 0;
    EIMSK &= ~(1 << INT0); // Временно отключаем

    OCR1B = TCNT1 + timer_ticks_per_bit + (timer_ticks_per_bit / 2);
    TIFR1 |= (1 << OCF1B);
    TIMSK1 |= (1 << OCIE1B);
}

// --- RX Sampling ISR ---
ISR(TIMER1_COMPB_vect) {
    OCR1B += timer_ticks_per_bit;

    if (rx_bit_index < 8) {
        if (PIND & (1 << UART_RX_PIN)) rx_byte_received |= (1 << rx_bit_index);
        rx_bit_index++;
    } else {
        if (PIND & (1 << UART_RX_PIN)) { // Стоп-бит HIGH
            uint8_t next_head = (rx_head + 1) % BUFFER_SIZE;
            if (next_head != rx_tail) {
                rx_buffer[rx_head] = rx_byte_received;
                rx_head = next_head;
            }
        }
        rx_active = false;
        TIMSK1 &= ~(1 << OCIE1B);
        EIFR |= (1 << INTF0);
        EIMSK |= (1 << INT0);
    }
}

void setup() {
    uart_set_baudrate(9600);
    Serial.begin(9600);
    sei();
    uart_send_string("UART Ready.\r\n");
}

char input_buffer[BUFFER_SIZE];
uint8_t buffer_index = 0;

void loop() {
    while (uart_available()) {
        char c = uart_read();
        Serial.write(c);
        uart_send(c);
        if (c == '\n') uart_send('\r');
    }

    static uint32_t last_send_time = 0;
    if (!tx_active && uart_available() == 0 && millis() - last_send_time > 5000) {
        uart_send_string("Test packet MCU TX.\r\n");
        last_send_time = millis();
    }
}
