#include <Arduino.h>
#include "Filters.h"
#define ENC_PIN PB6
#define INA_PIN PB7
#define INB_PIN PB8
#define PWM_PIN PB9

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

void setup()
{
  pinMode(ENC_PIN, INPUT);

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, 1);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PA3, OUTPUT);
  // pinMode(PB9,PWM);
  // digitalWrite(PB7,0);
  // digitalWrite(PB8,1);
  // analogWrite(PB9,255);/
  Serial.begin(9600);
  // analogWriteFrequency(1000);

  // motorDrive(0, 1, 200);

  attachInterrupt(digitalPinToInterrupt(ENC_PIN), readRPM, RISING);

  // analogWrite(PB9,100);
  // pwmWrite();
}
uint32_t x;
void loop()
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