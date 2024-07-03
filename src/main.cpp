#include <Arduino.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include "constants.h"
#include "bluetooth.h"
#include "motion.h"
// #include "imu.h"
#include "constants.h"

JointServo *servo[12];

MotionController motionController;
BluetoothSerialController btController;
// IMU imu;

void Task1(void *pvParameters);
void Task2(void *pvParameters);

void setup()
{
  Serial.begin(115200);

  // imu.setup(INTERRUPT_PIN);
  // Serial.println("IMU setup done");
  motionController.setup(SERVO_PINS, SERVO_OFFSETS);
  btController.setup(motionController);

  motionController.init();
  btController.begin();
  Serial.println("Bluetooth setup done");

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, NULL, 1);
}

void loop()
{
}

void Task1(void *pvParameters)
{
  motionController.reset();
  while (1)
  {
    vTaskDelay(20);
    motionController.setAutoAvoidance(true);
    // btController.receive();
  }
}

void Task2(void *pvParameters)
{
  while (1)
  {
    vTaskDelay(15);
    motionController.servoServe();
  }
}