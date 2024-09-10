#include <avr/io.h>
#include <avr/interrupt.h>

#define MOTOR_1_DIR_PIN 2
#define MOTOR_1_PUL_POS_PIN 3
#define MOTOR_2_DIR_PIN 4
#define MOTOR_2_PUL_POS_PIN 5
#define RAMP_PERIOD_START_MOTOR_1 800
#define RAMP_PERIOD_END_MOTOR_1 50
#define RAMP_PERIOD_START_MOTOR_2 900
#define RAMP_PERIOD_END_MOTOR_2 5
#define PERIOD_MOTOR_2 5
#define PERIOD_MOTOR_1 800
#define ONE_REV 25000
#define QUARTER_REV 25000/4
#define RUN_PULSES PULSE_PER_REV
#define CONFIG_LED_PIN 13
#define PWR_SW_1 7
#define LIMIT_SW_1 8
#define LIMIT_SW_2 10

#define PWM_PIN_A PB1       // Pin 9 (OC1A, Timer1)
#define PWM_PIN_B PB2       // Pin 10 (OC1B, Timer1)
#define PWM_PIN_C PD3       // Pin 3 (OC2B, Timer2)

#define DIRECTION_PIN_A PB3 // Pin 11 for Motor 1 direction
#define DIRECTION_PIN_B PB4 // Pin 12 for Motor 2 direction
#define DIRECTION_PIN_C PD2 // Pin 2 for Motor 3 direction

#define LIMIT_SWITCH_1 PD4  // Pin 4 for Limit Switch 1
#define LIMIT_SWITCH_2 PD5  // Pin 5 for Limit Switch 2

#define PRESCALER_TIMER1 256
#define PRESCALER_TIMER2 256

volatile uint16_t pulse_count_a = 0;
volatile uint16_t pulse_count_b = 0;
volatile uint16_t pulse_count_c = 0;

void timer1_init(uint16_t frequency_hz) {
    uint16_t top_value = (F_CPU / (PRESCALER_TIMER1 * frequency_hz)) - 1;
    TCCR1A = (1 << WGM11);  // Fast PWM, Mode 14 (ICR1 as TOP)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);  // Fast PWM, Prescaler 256
    ICR1 = top_value;
}

void pwm_start_timer1(uint8_t pin, uint8_t duty_cycle, uint16_t frequency_hz) {
    timer1_init(frequency_hz);
    uint16_t pulse_width = (ICR1 * duty_cycle) / 100;

    if (pin == PWM_PIN_A) {
        DDRB |= (1 << PWM_PIN_A);
        TCCR1A |= (1 << COM1A1);
        OCR1A = pulse_width;
    } else if (pin == PWM_PIN_B) {
        DDRB |= (1 << PWM_PIN_B);
        TCCR1A |= (1 << COM1B1);
        OCR1B = pulse_width;
    }
}

void pwm_stop_timer1(uint8_t pin) {
    if (pin == PWM_PIN_A) {
        TCCR1A &= ~(1 << COM1A1);  // Disable PWM on OC1A (Pin 9)
    } else if (pin == PWM_PIN_B) {
        TCCR1A &= ~(1 << COM1B1);  // Disable PWM on OC1B (Pin 10)
    }
}

void pwm_ramp_up_timer1(uint8_t pin, uint8_t duty_cycle, uint16_t start_frequency, uint16_t target_frequency, uint16_t total_steps) {
    uint16_t frequency_step = (target_frequency - start_frequency) / total_steps;
    uint16_t current_frequency = start_frequency;
    pulse_count_a = 0;
    pulse_count_b = 0;
    for (uint16_t step = 0; step < total_steps; step++) {
        current_frequency += frequency_step;
        pwm_start_timer1(pin, duty_cycle, current_frequency);
        while ((pin == PWM_PIN_A ? pulse_count_a : pulse_count_b) < step) {}
    }
}

void pwm_ramp_down_timer1(uint8_t pin, uint8_t duty_cycle, uint16_t start_frequency, uint16_t end_frequency, uint16_t total_steps) {
    uint16_t frequency_step = (start_frequency - end_frequency) / total_steps;
    uint16_t current_frequency = start_frequency;
    pulse_count_a = 0;
    pulse_count_b = 0;
    for (uint16_t step = 0; step < total_steps; step++) {
        current_frequency -= frequency_step;
        pwm_start_timer1(pin, duty_cycle, current_frequency);
        while ((pin == PWM_PIN_A ? pulse_count_a : pulse_count_b) < step) {}
    }
    pwm_stop_timer1(pin);  // Stop PWM after ramping down
}

void timer2_init(uint16_t frequency_hz) {
    uint8_t top_value = (F_CPU / (PRESCALER_TIMER2 * frequency_hz)) - 1;
    TCCR2A = (1 << WGM21) | (1 << WGM20);  // Fast PWM, Mode 3
    TCCR2B = (1 << CS22) | (1 << CS22) | (1 << CS21);  // Prescaler 256
    OCR2A = top_value;
}

void pwm_start_timer2(uint8_t duty_cycle, uint16_t frequency_hz) {
    timer2_init(frequency_hz);
    uint8_t pulse_width = (OCR2A * duty_cycle) / 100;

    DDRD |= (1 << PWM_PIN_C);  // Set PD3 (Pin 3, OC2B) as output
    TCCR2A |= (1 << COM2B1);   // Enable non-inverting PWM on OC2B (Pin 3)
    OCR2B = pulse_width;
}

void pwm_stop_timer2() {
    TCCR2A &= ~(1 << COM2B1);  // Disable PWM on OC2B (Pin 3)
}

void pwm_ramp_up_timer2(uint8_t duty_cycle, uint16_t start_frequency, uint16_t target_frequency, uint16_t total_steps) {
    uint16_t frequency_step = (target_frequency - start_frequency) / total_steps;
    pulse_count_c = 0;
    for (uint16_t step = 0; step < total_steps; step++) {
        uint16_t current_frequency = start_frequency + (step * frequency_step);
        pwm_start_timer2(duty_cycle, current_frequency);
        while (pulse_count_c < step) {}
    }
}

void pwm_ramp_down_timer2(uint8_t duty_cycle, uint16_t start_frequency, uint16_t end_frequency, uint16_t total_steps) {
    uint16_t frequency_step = (start_frequency - end_frequency) / total_steps;
    pulse_count_c = 0;
    for (uint16_t step = 0; step < total_steps; step++) {
        uint16_t current_frequency = start_frequency - (step * frequency_step);
        pwm_start_timer2(duty_cycle, current_frequency);
        while (pulse_count_c < step) {}
    }
    pwm_stop_timer2();  // Stop PWM after ramping down
}

void run_motor(uint8_t motor, uint16_t total_steps, uint16_t max_frequency, bool direction) {
    uint8_t duty_cycle = 75;  // Example duty cycle
    
    // Split the total steps into ramp-up and ramp-down steps
    uint16_t ramp_steps = total_steps / 2;
    uint16_t hold_steps = total_steps - 2 * ramp_steps;  // Steps at max frequency
    
    switch (motor) {
        case 1: // Motor 1
            pinMode(DIRECTION_PIN_A, OUTPUT);
            digitalWrite(DIRECTION_PIN_A, direction);
            
            // Ramp up
            pwm_ramp_up_timer1(PWM_PIN_A, duty_cycle, 0, max_frequency, ramp_steps);
            
            // Hold at max frequency
            pwm_start_timer1(PWM_PIN_A, duty_cycle, max_frequency);
            delay(hold_steps * 100);  // Delay for the hold time (adjust as needed)
            
            // Ramp down
            pwm_ramp_down_timer1(PWM_PIN_A, duty_cycle, max_frequency, 0, ramp_steps);
            break;
        
        case 2: // Motor 2
            pinMode(DIRECTION_PIN_B, OUTPUT);
            digitalWrite(DIRECTION_PIN_B, direction);
            
            // Ramp up
            pwm_ramp_up_timer1(PWM_PIN_B, duty_cycle, 0, max_frequency, ramp_steps);
            
            // Hold at max frequency
            pwm_start_timer1(PWM_PIN_B, duty_cycle, max_frequency);
            delay(hold_steps * 100);  // Delay for the hold time (adjust as needed)
            
            // Ramp down
            pwm_ramp_down_timer1(PWM_PIN_B, duty_cycle, max_frequency, 0, ramp_steps);
            break;
        
        case 3: // Motor 3
            pinMode(DIRECTION_PIN_C, OUTPUT);
            digitalWrite(DIRECTION_PIN_C, direction);
            
            // Ramp up
            pwm_ramp_up_timer2(duty_cycle, 0, max_frequency, ramp_steps);
            
            // Hold at max frequency
            pwm_start_timer2(duty_cycle, max_frequency);
            delay(hold_steps * 100);  // Delay for the hold time (adjust as needed)
            
            // Ramp down
            pwm_ramp_down_timer2(duty_cycle, max_frequency, 0, ramp_steps);
            break;
    }
}

ISR(TIMER1_OVF_vect) {
    pulse_count_a++;
    pulse_count_b++;
}

ISR(TIMER2_OVF_vect) {
    pulse_count_c++;
}

void setup() {
    pinMode(PWM_PIN_A, OUTPUT);     // Set PWM pins as outputs
    pinMode(PWM_PIN_B, OUTPUT);
    pinMode(PWM_PIN_C, OUTPUT);
    pinMode(LIMIT_SWITCH_1, INPUT_PULLUP); // Set limit switch pins as inputs
    pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);

    // Enable Timer1 and Timer2 overflow interrupts
    TIMSK1 = (1 << TOIE1);  // Timer1 overflow interrupt enable
    TIMSK2 = (1 << TOIE2);  // Timer2 overflow interrupt enable

    // Enable global interrupts
    sei();

    // Example: Run motor 1
    run_motor(1, 10, 200, true);  // Run Motor 1 with 10 steps, max frequency 200 Hz, forward direction
}

void loop() {
    // // Example of handling limit switches for motor 3
    // if (digitalRead(LIMIT_SWITCH_1) == LOW) {
    //     run_motor(3, 10, 300, false);  // Example: Run Motor 3 in reverse direction
    // }

    // if (digitalRead(LIMIT_SWITCH_2) == LOW) {
    //     run_motor(3, 10, 300, true);  // Example: Run Motor 3 in forward direction
    // }
    run_motor(3, 500, 10, false);
    delay(2000);
    run_motor(3, 500, 10, true);
    delay(2000);
}
