#include <avr/io.h>
#include <avr/interrupt.h>

// Используем PORTB, что соответствует пинам 8-12 на Arduino Uno
#define LED_PORT PORTB
#define LED_DDR  DDRB

const uint8_t PINS[] = {
    (1 << PB4), // LED A (D12)
    (1 << PB3), // LED B (D11)
    (1 << PB2), // LED C (D10)
    (1 << PB1), // LED D (D9)
    (1 << PB0)  // LED E (D8)
};
#define NUM_LEDS 5

const unsigned int PERIODS[] = {1000, 2000, 3000, 4000, 5000}; 

volatile unsigned int master_counter = 0; // Общий счетчик тактов
volatile unsigned int led_counters[NUM_LEDS] = {0}; // Счетчики для каждого LED


void update_leds() {
    master_counter++;

    for (int i = 0; i < NUM_LEDS; i++) {
        led_counters[i]++;

        if (led_counters[i] >= PERIODS[i]) {
            LED_PORT ^= PINS[i];
            led_counters[i] = 0;
        }
    }
}

ISR(TIMER1_COMPA_vect) {
    update_leds();
}

void setup() {
    LED_DDR |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4);
    
    LED_PORT &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4));

    cli();

    OCR1A = 624; // 10 мс
    
    TCCR1A = 0;
    TCCR1B = 0;
    
    TCCR1B |= (1 << WGM12);
    
    TCCR1B |= (1 << CS12);
    
    TIMSK1 |= (1 << OCIE1A);

    sei();
}


void loop() {
}

