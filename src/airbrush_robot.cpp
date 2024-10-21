#include <stdint.h>
#include <Arduino.h>

#define CONFIG_LED_PIN 13

// Used when painting letters
#define CONFIG_MOTIVE_PAINT false

// Pin Definitions
#define SIGN_MOTOR_DIR_PIN 2
#define SIGN_MOTOR_PUL_POS_PIN 3
#define SWIPE_MOTOR_DIR_PIN 4
#define SWIPE_MOTOR_PUL_POS_PIN 5
#define PAINT_MOTOR_DIR_PIN 12
#define PAINT_MOTOR_PUL_POS_PIN 11
#define PWR_SW_1 7
#define PWR_SW_PWR 6
#define LIMIT_SW_2 8
#define LIMIT_SW_1 10

// Timing Constants
#define PERIOD_SIGN_MOTOR 100
#define PERIOD_SWIPE_MOTOR 20
#define PERIOD_SWIPE_MOTOR_INIT 200
#define PERIOD_PAINT_MOTOR 100
#define PAINT_BUTTON_STEPS 500
#define PAINT_BUTTON_INITIAL_STEPS 700
#define SIGN_ROTATE_STEPS 6250
#define SIGN_MOTIVE_ROTATE_STEPS 0
#define SWIPE_MOTOR_INIT_STEPS 1500
#define PAINT_DELAY 5000

// Ramping Constants
#define RAMP_UP_STEP 0.008
#define RAMP_DWN_STEP 0.02

// Enumerations for direction and paint state
enum swipe_direction { LEFT = true, RIGHT = false };
enum paint { ON = false, OFF = true };

// Motor Direction Control
void set_motor_direction(uint8_t dir_pin, bool direction) {
    digitalWrite(dir_pin, direction);
}

// Motor Pulse Control
void pulse_motor(int pin, long pulse_period, int steps) {
    for (int i = 0; i < steps; i++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulse_period);
        digitalWrite(pin, LOW);
        delayMicroseconds(pulse_period);
    }
}

// Setup Function
void setup_pins() {
    pinMode(SIGN_MOTOR_DIR_PIN, OUTPUT);
    pinMode(SWIPE_MOTOR_DIR_PIN, OUTPUT);
    pinMode(PAINT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(SIGN_MOTOR_PUL_POS_PIN, OUTPUT);
    pinMode(SWIPE_MOTOR_PUL_POS_PIN, OUTPUT);
    pinMode(PAINT_MOTOR_PUL_POS_PIN, OUTPUT);
    
    pinMode(PWR_SW_PWR, OUTPUT);
    digitalWrite(PWR_SW_PWR, LOW);
    pinMode(PWR_SW_1, INPUT_PULLUP);
    
    pinMode(LIMIT_SW_1, INPUT);
    pinMode(LIMIT_SW_2, INPUT);

    digitalWrite(PWR_SW_PWR, LOW);
}

// Start and Stop Paint Motor
void start_paint_motor(int steps, long period) {
    set_motor_direction(PAINT_MOTOR_DIR_PIN, ON);
    pulse_motor(PAINT_MOTOR_PUL_POS_PIN, period, steps);
}

void stop_paint_motor(int steps, long period) {
    set_motor_direction(PAINT_MOTOR_DIR_PIN, OFF);
    pulse_motor(PAINT_MOTOR_PUL_POS_PIN, period, steps);
}

// Rotate Sign Motor
void rotate_sign_motor(int steps, long period, bool direction) {
    set_motor_direction(SIGN_MOTOR_DIR_PIN, direction);
    pulse_motor(SIGN_MOTOR_PUL_POS_PIN, period, steps);
}

// Ramp-up Function
void run_ramp(int pin, float min_pulse_period, float max_pulse_period) {
    float pulse_period = max_pulse_period;

    // Ramp-up phase
    while (pulse_period > min_pulse_period) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulse_period);
        digitalWrite(pin, LOW);
        delayMicroseconds(pulse_period);
        pulse_period -= RAMP_UP_STEP;
    }

    // Constant speed
    while (true) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(min_pulse_period);
        digitalWrite(pin, LOW);
        delayMicroseconds(min_pulse_period);

        if (digitalRead(LIMIT_SW_1) || digitalRead(LIMIT_SW_2)) {
            pulse_period = min_pulse_period;
            while (pulse_period < max_pulse_period) {
                digitalWrite(pin, HIGH);
                delayMicroseconds(pulse_period);
                digitalWrite(pin, LOW);
                delayMicroseconds(pulse_period);
                pulse_period += RAMP_DWN_STEP;
            }
            break;
        }
    }
}

// Main Robot Functionality
void run_robot_sign() {
    if (digitalRead(PWR_SW_1)) {
        run_ramp(SWIPE_MOTOR_PUL_POS_PIN, 18, 38);
        if (digitalRead(LIMIT_SW_1)) 
        {
          set_motor_direction(SWIPE_MOTOR_DIR_PIN, LEFT);
          stop_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          rotate_sign_motor(SIGN_ROTATE_STEPS, PERIOD_SIGN_MOTOR, true);
          delay(PAINT_DELAY);
          if (digitalRead(PWR_SW_1))
          {
            start_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          }
        }

        if (digitalRead(LIMIT_SW_2)) {
          set_motor_direction(SWIPE_MOTOR_DIR_PIN, RIGHT);
          stop_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          rotate_sign_motor(SIGN_ROTATE_STEPS, PERIOD_SIGN_MOTOR, true);
          delay(PAINT_DELAY);
          if (digitalRead(PWR_SW_1))
          {
            start_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          }
        }
        
    }
}

// Main Robot Functionality
void run_robot_motive() {
    if (digitalRead(PWR_SW_1)) {
        run_ramp(SWIPE_MOTOR_PUL_POS_PIN, 18, 38);
        if (digitalRead(LIMIT_SW_1)) 
        {
          set_motor_direction(SWIPE_MOTOR_DIR_PIN, LEFT);
          set_motor_direction(SIGN_MOTOR_DIR_PIN, LEFT);
          stop_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          rotate_sign_motor(SIGN_MOTIVE_ROTATE_STEPS, PERIOD_SIGN_MOTOR, true);
          delay(PAINT_DELAY);
          if (digitalRead(PWR_SW_1))
          {
            start_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          }
        }

        if (digitalRead(LIMIT_SW_2)) {
          set_motor_direction(SWIPE_MOTOR_DIR_PIN, RIGHT);
          set_motor_direction(SIGN_MOTOR_DIR_PIN, RIGHT);
          stop_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          rotate_sign_motor(SIGN_MOTIVE_ROTATE_STEPS, PERIOD_SIGN_MOTOR, true);
          delay(PAINT_DELAY);
          if (digitalRead(PWR_SW_1))
          {
            start_paint_motor(PAINT_BUTTON_STEPS, PERIOD_PAINT_MOTOR);
          }
        }
        
    }
}

// Move to Start Position
void go_to_start_position() {
    if (!digitalRead(LIMIT_SW_2))
    {
      set_motor_direction(SWIPE_MOTOR_DIR_PIN, LEFT);
      run_ramp(SWIPE_MOTOR_PUL_POS_PIN, 18, 38);
      Serial.println("Init ramp");
      set_motor_direction(SWIPE_MOTOR_DIR_PIN, RIGHT);
    }
    
    Serial.println("pulse motor");
    pulse_motor(SWIPE_MOTOR_PUL_POS_PIN, PERIOD_SWIPE_MOTOR_INIT, SWIPE_MOTOR_INIT_STEPS);
    while(!digitalRead(PWR_SW_1));
    start_paint_motor(PAINT_BUTTON_INITIAL_STEPS, PERIOD_PAINT_MOTOR);
}

void setup() {
    Serial.begin(9600);
    setup_pins();
    go_to_start_position();
}

void loop() {
    #if(CONFIG_MOTIVE_PAINT == true)
    run_robot_motive();
    #else
    run_robot_sign();
    #endif
    
}
