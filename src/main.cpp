#include <Arduino.h>
#include "driver/mcpwm.h"

#include <soc/mcpwm_reg.h>


#include "DriverSettings.h"
#include "MagneticEncoder.h"
#include <SPI.h>
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <stdio.h>
#include <string.h>
#include "freertos/task.h"
#include "freertos/semphr.h"



const int LO1 = 14; // Pin used for PWM output
const int HO1 = 12; // Pin used for PWM output

const int LO2 = 26; // Pin used for PWM output
const int HO2 = 27; // Pin used for PWM output

const int LO3 = 33; // Pin used for PWM output
const int HO3 = 25;

const int csPin = 5;
const int EN_GATE = 15;

#define PHASE_DELAY_1 (double)2.094395102 // 120°
#define PHASE_DELAY_2 (double)4.188790205 // 240°

const float MAX_POSITION = 359.9 * PI / 180;
const float MIN_POSITION = 0.0;
int POLE_PAIRS = 21;

const int ledPin = 22; // GPIO 22
// volatile bool ledState = LOW;
// volatile bool interruptFlag = false;

const float freqCarr = 19200;            // Carrier freq.
float freqMod = 50.0;                    // Modulation Frequency Hz
int amplitude = 300;                     // max counter/timer value
int sampleNum = int(freqCarr / freqMod); //
int i;

float angleDiff;
float angleDiffRad;
float direction = 0;
float currentPos;

SemaphoreHandle_t myMutex;
// test github

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
#define MOTOR_TASK_FREQUENCY_HZ 200   // Motor control task frequency in Hertz
#define ENCODER_TASK_FREQUENCY_HZ 200 // Encoder reading task frequency in Hertz
int DEADTime =5;

SemaphoreHandle_t xPosMutex = xSemaphoreCreateMutex();


void encoderReadingTask(void *parameter)
{
  const TickType_t encoderTaskPeriod = pdMS_TO_TICKS(1000 / ENCODER_TASK_FREQUENCY_HZ);
  TickType_t encoderLastWakeTime = xTaskGetTickCount();


  while (1)
  {

    if(xSemaphoreTake(xPosMutex, portMAX_DELAY) == pdTRUE) {
      currentPos = readAngle1(csPin);
      xSemaphoreGive(xPosMutex); // Release mutex
    }

    // Process encoder value as needed
    // Serial.print("Encoder Angle: ");
    // Serial.println(currentPos);

    // Delay until the next encoder task period
    vTaskDelayUntil(&encoderLastWakeTime, encoderTaskPeriod);
    
  }
 
}


void motorControlTask(void *parameter)
{

  const TickType_t motorTaskPeriod = pdMS_TO_TICKS(1000 / MOTOR_TASK_FREQUENCY_HZ);
  TickType_t motorLastWakeTime = xTaskGetTickCount();

  while (1)
  {

     if(xSemaphoreTake(xPosMutex, portMAX_DELAY) == pdTRUE) {
      // Use currentPos


      xSemaphoreGive(xPosMutex); // Release mutex
    }

    // Set duty cycles for MCPWM channels
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 20);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 20);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 20);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 20);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 20);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 20);

    // Delay until the next motor task period
    vTaskDelayUntil(&motorLastWakeTime, motorTaskPeriod);
  }
}



float calculateShortestError(float currentAngle, float commandedAngle) {
    const float twoPi = 2 * PI;
    
    // Normalize angles to be within the range [0, 2*pi)
    currentAngle = fmodf(currentAngle, twoPi);
    commandedAngle = fmodf(commandedAngle, twoPi);

    // Calculate the angular distance in both directions
    float clockwiseDistance = fmodf(commandedAngle - currentAngle + twoPi, twoPi);
    float counterclockwiseDistance = fmodf(currentAngle - commandedAngle + twoPi, twoPi);

    // Check for a potential jump across zero degrees
    if (abs(commandedAngle - currentAngle) > PI) {
        // Adjust the distances to consider the jump
        clockwiseDistance = twoPi - clockwiseDistance;
        counterclockwiseDistance = twoPi - counterclockwiseDistance;
    }

    // Choose the shortest distance
    float shortestDistance = min(clockwiseDistance, counterclockwiseDistance);

    // Determine the direction (sign) of the error
    float error = (clockwiseDistance < counterclockwiseDistance) ? shortestDistance : -shortestDistance;

    return error;
}


void setup()
{

  Serial.begin(115200);

  SPI.begin();




  // put your setup code here, to run once:
  setCpuFrequencyMhz(240);

  // delay(1000);

  xTaskCreate(encoderReadingTask, "EncoderReadingTask", 4096, NULL, 1, NULL);

    //  xTaskCreate(encoderReadingTask, "MotorControlTask", 4096, NULL, 1, NULL);


  xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask", 4096, NULL, 2, NULL, 0);

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH); // Deselect AS5048A initially

  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, HIGH);

  pinMode(ledPin, OUTPUT);

  mcpwm_config_t pwm_config_A;
  pwm_config_A.frequency = freqCarr * 2;             // Set frequency in Hz
  pwm_config_A.counter_mode = MCPWM_UP_DOWN_COUNTER; // phase freq correct
  pwm_config_A.duty_mode = MCPWM_DUTY_MODE_0;        // active high PWM
  pwm_config_A.cmpr_a = 0.0;                         // duty cycle to 0%
  pwm_config_A.cmpr_b = 0.0;


  pinMode(LO1, OUTPUT);
  pinMode(HO1, OUTPUT);

  pinMode(LO2, OUTPUT);
  pinMode(HO2, OUTPUT);

  pinMode(LO3, OUTPUT);
  pinMode(HO3, OUTPUT);

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_A); // init PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO1);            // Use GPIO 12 for MCPWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO1);            // Use GPIO 13 for MCPWM0B

  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, DEADTime, DEADTime);

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_A); // init PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO2);            // Use GPIO 12 for MCPWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO2);            // Use GPIO 13 for MCPWM0B

  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, DEADTime, DEADTime);

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config_A); // init PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO3);            // Use GPIO 12 for MCPWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO3);            // Use GPIO 13 for MCPWM0B

  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, DEADTime, DEADTime);

  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);

 
 
}

void loop()
{

//  vTaskDelay(1000);

}



// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}