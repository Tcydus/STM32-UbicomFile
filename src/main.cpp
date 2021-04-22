#include <Arduino.h>
#include "Filters.h"

#define ENC_PIN PB6

#define INA_PIN PB7
#define INB_PIN PB8
#define PWM_PIN PB9

#define LIM_B_PIN PA0
#define LIM_F_PIN PA1
#define SW1_PIN PA2
#define LDR_PIN PA3
#define RAIN_PIN PA4

#define LED1_PIN PB12
#define LED2_PIN PB13

FilterOnePole Lowpass(LOWPASS, 0.6);
float motor_rpm = 0.0f;
float motor_rpm_no_filter = 0.0f;
float pid_value;

float intergral_error = 0.0f, last_error = 0.0f;

void readRPM();
void motorDrive(bool ina_val, bool inb_val, uint8_t pwm_val);
void speedControl(uint16_t target_speed, String direction);
void zeroRPMCheck();
int smooth(int data, float filterVal, float smoothedVal);
void debugLoop();
void task1();

void setup()
{
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, 1);

  pinMode(ENC_PIN, INPUT);

  pinMode(INA_PIN, OUTPUT);
  pinMode(INB_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  pinMode(LIM_B_PIN, INPUT);
  pinMode(LIM_F_PIN, INPUT);
  pinMode(SW1_PIN, INPUT);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  Serial.begin(9600);
  // analogWriteFrequency(1000);

  attachInterrupt(digitalPinToInterrupt(ENC_PIN), readRPM, RISING);
}
uint32_t x;
void loop()
{

  ///////////////////////// main code ///////////////////
  static uint32_t last_time_interval = 0;

  if (millis() - last_time_interval >= 10)
  {
    task1();
  }

  ///////////////////////// See a value ///////////////////
  //debugLoop();
}

void debugLoop()
{

  static uint32_t last_time_interval = 0;
  static uint32_t last_motor_interval = 0;
  static bool motor_en = true;

  if (millis() - last_motor_interval >= 5000)
  {
    last_motor_interval = millis();
    motor_en = !motor_en;
    speedControl(0, "NONE");
    delay(2000);
  }

  if (millis() - last_time_interval >= 10)
  {
    zeroRPMCheck();
    // speedControl(150,"CW");
    // int spd;
    //   if(motor_en){
    //       spd = 130;
    //   }else{
    //       spd = 0;

    //   }
    if (motor_en)
    {

      speedControl(150, "CW");
    }
    else
    {
      speedControl(150, "CCW");
    }

    last_time_interval = millis();
    String text = String(pid_value);
    text += " ";
    text += String(motor_rpm_no_filter);
    text += " ";
    text += String(motor_rpm);
    text += " ";
    text += String(digitalRead(ENC_PIN));
    Serial.println(text);
    // motorDrive(0, 1, spd);
  }
}

void task1()
{

  // 1 2 OUTSIDE TO_INSIDE
  enum mode
  {
    AUTO,
    MANUAL
  };

  enum status
  {
    INSIDE,
    OUTSIDE
  };
  enum direction
  {
    DO_NOTHING,
    TO_OUTSIDE,
    TO_INSIDE

  };

  static status clothes_rack = INSIDE;
  static direction rail_direction = DO_NOTHING;
  static mode system_mode = AUTO;
  // so dark so high resistance ////ldr
  const uint16_t ldr_high = 700, ldr_low = 300; //dim avg480 light
  const uint16_t rain_high = 700;
  static bool led1_val = 1, led2_val = 1;

  digitalWrite(LED1_PIN, led1_val);
  digitalWrite(LED2_PIN, led2_val);

  if (digitalRead(LIM_F_PIN))
  {
    clothes_rack = INSIDE;
  }
  else if (digitalRead(LIM_B_PIN))
  {
    clothes_rack = OUTSIDE;
  }

  if (!digitalRead(SW1_PIN))
  {
    delay(50);
    if (!digitalRead(SW1_PIN))
    {
      if (system_mode == MANUAL)
      {
        clothes_rack == INSIDE ? rail_direction = TO_OUTSIDE : rail_direction = TO_INSIDE;
      }
    }
    uint32_t last_time = millis();
    while (!digitalRead(SW1_PIN))
    {
      if (millis() - last_time >= 3000)
      {
        if (system_mode == AUTO)
        {
          system_mode = MANUAL;
          rail_direction = DO_NOTHING;
          led1_val = 1;
          led2_val = 0;
        }
        else if (system_mode == MANUAL)
        {
          system_mode = AUTO;
          rail_direction = DO_NOTHING;
          led1_val = 1;
          led2_val = 1;
        }
        digitalWrite(LED1_PIN, led1_val);
        digitalWrite(LED2_PIN, led2_val);
        while (!digitalRead(SW1_PIN))
          ;
        break;
      }
    }
  }

  if (system_mode == AUTO)
  {

    if (analogRead(LDR_PIN) <= ldr_low)
    {
      rail_direction = TO_OUTSIDE;
    }
    else if (analogRead(LDR_PIN) >= ldr_high)
    {
      rail_direction = TO_INSIDE;
    }

    if (analogRead(RAIN_PIN) <= rain_high)
    {
      rail_direction = TO_INSIDE;
    }
  }

  if (rail_direction == TO_OUTSIDE && clothes_rack == OUTSIDE)
  {
    rail_direction = DO_NOTHING;
  }
  else if (rail_direction == TO_INSIDE && clothes_rack == INSIDE)
  {
    rail_direction = DO_NOTHING;
  }

  Serial.print(analogRead(LDR_PIN));
  Serial.print(" ");
  Serial.print(analogRead(RAIN_PIN));
  Serial.print(" ");
  Serial.print(clothes_rack);
  Serial.print(" ");
  Serial.println(rail_direction);

  if (rail_direction == TO_OUTSIDE)
  {
    speedControl(150, "CCW");
  }
  else if (rail_direction == TO_INSIDE)
  {
    speedControl(150, "CW");
  }
  else if (rail_direction == DO_NOTHING)
  {
    speedControl(0, "NONE");
  }
}
void readRPM()
{
  static uint32_t last_time = 0;
  const uint32_t PULSE_PER_ROUND = 20, TO_SEC = 1000000, TO_MINUTE = 60, LOWER_GEARBOX = 24;

  uint32_t period = micros() - last_time;

  motor_rpm_no_filter = (float)1 / (float)(period * PULSE_PER_ROUND) * TO_SEC * TO_MINUTE / LOWER_GEARBOX;
  Lowpass.input(motor_rpm_no_filter);
  motor_rpm = Lowpass.output();

  last_time = micros();
}

void motorDrive(bool ina_val, bool inb_val, uint8_t pwm_val)
{
  // const uint8_t MIN_PWM = 100, MAX_PWM = 255;
  digitalWrite(INA_PIN, ina_val);
  digitalWrite(INB_PIN, inb_val);
  // pwm_val = constrain(pwm_val, MIN_PWM, MAX_PWM);
  analogWrite(PWM_PIN, pwm_val);
}

void speedControl(uint16_t target_speed, String direction)
{
  // const float KP = 2.0f, KI = 0.05f, KD = 1.2f, MAX_ERROR = 10000;
  //  const float KP = 1.8f, KI = 0.00f, KD = 0.07f, MAX_ERROR = 10000;
  const float KP = 1.2f, KI = 0.003f, KD = 0.07f, MAX_ERROR = 10000;

  float error = target_speed - motor_rpm;
  intergral_error += error;
  intergral_error = constrain(intergral_error, -MAX_ERROR, MAX_ERROR);
  float diff_error = error - last_error;
  last_error = error;

  pid_value = KP * error + KI * intergral_error + KD * diff_error;
  pid_value = constrain(pid_value, 50, 255);

  if (target_speed == 0)
    pid_value = 0;

  bool cw_val = 0, ccw_val = 0;

  if (direction == "CCW")
  {
    cw_val = 1;
    ccw_val = 0;
  }
  else if (direction == "CW")
  {
    cw_val = 0;
    ccw_val = 1;
  }

  motorDrive(cw_val, ccw_val, pid_value);
}

void zeroRPMCheck()
{
  static float motor_rpm_temp = motor_rpm;
  static uint32_t last_time;
  // if(motor_rpm == motor_rpm_temp){
  //   if(millis() - last_time  >= 1000){
  //     motor_rpm = 0;
  //   }
  // }
  // else{
  //   motor_rpm_temp = motor_rpm;
  //   last_time = millis();
  // }
  if (millis() - last_time >= 100)
  {
    last_time = millis();
    if (motor_rpm == motor_rpm_temp)
    {
      motor_rpm_no_filter = 0;
      motor_rpm = 0;
    }
    else
    {
      motor_rpm_temp = motor_rpm;
    }
  }
}