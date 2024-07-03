#include <Arduino.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include "tcp.h"
#include "motion.h"
#include "imu.h"

JointServo *servo[12];

MotionController motionController;
TCPController tcpController;
IMU imu;

void Task1(void *pvParameters);
void Task2(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 0, 26);

  imu.setup(INTERRUPT_PIN);
  Serial.println("IMU setup done");
  motionController.setup(SERVO_PINS, SERVO_OFFSETS);
  tcpController.setup(motionController);

  motionController.init();
  tcpController.begin("ESP32-BLT-G24", "87654321");
  Serial.println("Bluetooth setup done");

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, NULL, 1);
}

void loop()
{
  imu.update();
  delay(20);
}

void Task1(void *pvParameters)
{
  motionController.reset();
  vTaskDelay(5000);
  while (1)
  {
    vTaskDelay(20);
    // motionController.setAutoAvoidance(true);
    tcpController.receive();
    // motionController.takePicture();
    // vTaskDelay(2000);
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