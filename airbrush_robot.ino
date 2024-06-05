
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

bool ramp_up = true;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_1_DIR_PIN, OUTPUT);
  pinMode(MOTOR_2_DIR_PIN, OUTPUT);
  pinMode(MOTOR_1_PUL_POS_PIN, OUTPUT);
  pinMode(MOTOR_2_PUL_POS_PIN, OUTPUT);

  pinMode(PWR_SW_1, INPUT_PULLUP);
  pinMode(LIMIT_SW_1, INPUT);
  pinMode(LIMIT_SW_2, INPUT);

  digitalWrite(MOTOR_1_DIR_PIN, HIGH);
  digitalWrite(MOTOR_2_DIR_PIN, HIGH);
  digitalWrite(MOTOR_1_PUL_POS_PIN, LOW);
  digitalWrite(MOTOR_2_PUL_POS_PIN, LOW);

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

void loop() {
  //run_steps_acceleration(QUARTER_REV, RAMP_PERIOD_START_MOTOR_1, RAMP_PERIOD_END_MOTOR_1, MOTOR_1_PUL_POS_PIN);
  //run_steps_acceleration(ONE_REV, RAMP_PERIOD_START_MOTOR_2, RAMP_PERIOD_END_MOTOR_2, MOTOR_2_PUL_POS_PIN);
  // while (digitalRead(PWR_SW_1) && !digitalRead(LIMIT_SW_1))
  // {   
  //   run(MOTOR_2_PUL_POS_PIN, PERIOD_MOTOR_2);
  // }
  if(digitalRead(LIMIT_SW_1))
  {
    digitalWrite(MOTOR_2_DIR_PIN, HIGH);
  }
  if(digitalRead(LIMIT_SW_2))
  {
    digitalWrite(MOTOR_2_DIR_PIN, LOW);
  }
  if(digitalRead(PWR_SW_1))
  {
    run(MOTOR_2_PUL_POS_PIN, PERIOD_MOTOR_2);
  }
  
  // Serial.print("Limit_SW_1: ");
  // Serial.println(digitalRead(LIMIT_SW_1));
  // Serial.print("Limit_SW_2: ");
  // Serial.println(digitalRead(LIMIT_SW_2));
  // Serial.print("Motor_1_DIR: ");
  // Serial.println(digitalRead(MOTOR_1_DIR_PIN));
  // Serial.print("Motor_2_DIR: ");
  // Serial.println(digitalRead(MOTOR_2_DIR_PIN));
  // Serial.println("Run motor"); 
  // Serial.println("");
  //delay(1000); 
}
