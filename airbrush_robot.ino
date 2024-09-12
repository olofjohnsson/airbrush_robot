#include <stdint.h>


#define MOTOR_1_DIR_PIN 2
#define MOTOR_1_PUL_POS_PIN 3
#define MOTOR_2_DIR_PIN 4
#define MOTOR_2_PUL_POS_PIN 5
#define MOTOR_3_DIR_PIN 9
#define MOTOR_3_PUL_POS_PIN 11
#define RAMP_PERIOD_START_MOTOR_1 800
#define RAMP_PERIOD_END_MOTOR_1 50
#define RAMP_PERIOD_START_MOTOR_2 900
#define RAMP_PERIOD_END_MOTOR_2 5
#define PERIOD_MOTOR_2 20
#define PERIOD_MOTOR_1 100
#define PERIOD_MOTOR_3 100
#define ONE_REV 25000
#define QUARTER_REV 25000/4
#define RUN_PULSES PULSE_PER_REV
#define CONFIG_LED_PIN 13
#define PWR_SW_1 7
#define PWR_SW_PWR 6
#define LIMIT_SW_2 8
#define LIMIT_SW_1 10

#define RAMP_STEP 0.008

bool ramp_up = true;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_1_DIR_PIN, OUTPUT);
  pinMode(MOTOR_2_DIR_PIN, OUTPUT);
  pinMode(MOTOR_3_DIR_PIN, OUTPUT);
  pinMode(MOTOR_1_PUL_POS_PIN, OUTPUT);
  pinMode(MOTOR_2_PUL_POS_PIN, OUTPUT);
  pinMode(MOTOR_3_PUL_POS_PIN, OUTPUT);

  pinMode(PWR_SW_1, INPUT_PULLUP);
  pinMode(PWR_SW_PWR, OUTPUT);
  pinMode(LIMIT_SW_1, INPUT);
  pinMode(LIMIT_SW_2, INPUT);

  digitalWrite(MOTOR_1_DIR_PIN, HIGH);
  digitalWrite(MOTOR_2_DIR_PIN, HIGH);
  digitalWrite(MOTOR_3_DIR_PIN, HIGH);
  digitalWrite(MOTOR_1_PUL_POS_PIN, LOW);
  digitalWrite(MOTOR_2_PUL_POS_PIN, LOW);
  digitalWrite(MOTOR_3_PUL_POS_PIN, LOW);
  digitalWrite(PWR_SW_PWR, LOW);

}

void run_steps_acceleration(int nr_of_steps, long ramp_start, long ramp_end, int pin)
{
  long pulse_period = ramp_start;
  for (int i=0;i<=nr_of_steps;i++)
  {
    if (pulse_period > ramp_end && ramp_up)
    {
      pulse_period--;
    }
    if (i>nr_of_steps-(ramp_start-ramp_end))
    {
      ramp_up = false;
      pulse_period++;
    }
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulse_period);
    digitalWrite(pin, LOW);
    delayMicroseconds(pulse_period);
  }
  ramp_up = true;
}

void run(int pin, long pulse_period)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulse_period);
  digitalWrite(pin, LOW);
  delayMicroseconds(pulse_period);
}

void run_ramp(int pin, float min_pulse_period, float max_pulse_period)
{
    float pulse_period = max_pulse_period;
    static bool up = true; 

    // Ramp-up phase
    while (pulse_period > min_pulse_period) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulse_period);
        digitalWrite(pin, LOW);
        delayMicroseconds(pulse_period);
        
        // Decrease the pulse period to increase speed (ramp-up)
        pulse_period -= RAMP_STEP;  // Adjust step size if necessary for smoother ramping
    }

    pulse_period = min_pulse_period;

    // Continue running at constant speed (min_pulse_period)
    while (true) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(min_pulse_period);
        digitalWrite(pin, LOW);
        delayMicroseconds(min_pulse_period);
        if (digitalRead(LIMIT_SW_1))
        {            
            while (pulse_period < max_pulse_period) 
            {
              digitalWrite(pin, HIGH);
              delayMicroseconds(pulse_period);
              digitalWrite(pin, LOW);
              delayMicroseconds(pulse_period);
        
            // Decrease the pulse period to increase speed (ramp-up)
            pulse_period += RAMP_STEP;  // Adjust step size if necessary for smoother ramping
            }
            delay(100);
            digitalWrite(MOTOR_2_DIR_PIN, HIGH);
            break;
        }
        if (digitalRead(LIMIT_SW_2))
        {
            while (pulse_period < max_pulse_period) 
            {
              digitalWrite(pin, HIGH);
              delayMicroseconds(pulse_period);
              digitalWrite(pin, LOW);
              delayMicroseconds(pulse_period);
        
            // Decrease the pulse period to increase speed (ramp-up)
            pulse_period += RAMP_STEP;  // Adjust step size if necessary for smoother ramping
            }
            delay(100);
            digitalWrite(MOTOR_2_DIR_PIN, LOW);
            break;
        }
    }
}

void toggle_motor_direction(uint8_t pin)
{
    static bool dir = LOW;
    digitalWrite(pin, dir);
    dir = !dir;
}

void run_air_brush_motor()
{
    if (digitalRead(LIMIT_SW_1))
    {
        digitalWrite(MOTOR_2_DIR_PIN, HIGH);
    }
    if (digitalRead(LIMIT_SW_2))
    {
        digitalWrite(MOTOR_2_DIR_PIN, LOW);
    }
    if (digitalRead(PWR_SW_1))
    {
        run_ramp(MOTOR_2_PUL_POS_PIN, 18, 38);
    }
}

void loop()
{
  run_air_brush_motor();
}
