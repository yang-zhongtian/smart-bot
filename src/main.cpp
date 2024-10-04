#include <Arduino.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <ESPmDNS.h>
#include "tcp.h"
#include "motion.h"
#include "imu.h"

const char *ssid = "G24-ESP32-AP";
const char *password = "88888888";

const char *server = "esp32-bot";
const char *service = "driver";

JointServo *servo[12];

MotionController motionController;
TCPController tcpController;
IMU imu;

void Task1(void *pvParameters);
void Task2(void *pvParameters);
void Task3(void *pvParameters);
void Task4(void *pvParameters);
void Task5(void *pvParameters);

TaskHandle_t task4Handle = NULL;

void setup()
{
  Serial.begin(115200);

  imu.setup(INTERRUPT_PIN);
  Serial.println("IMU setup done");
  motionController.setup(SERVO_PINS, SERVO_OFFSETS);
  tcpController.setup(motionController, &task4Handle);

  motionController.init();
  Serial.println("MotionController setup done");
  tcpController.begin(ssid, password);
  MDNS.begin(server);

  Serial.println("Setup done");

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Task2, "Task1", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Task3, "Task3", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task4, "Task4", 10000, NULL, 1, &task4Handle, 1);
  xTaskCreatePinnedToCore(Task5, "Task5", 10000, NULL, 1, NULL, 1);

  MDNS.addService(service, "tcp", TCP_SERVER_PORT);
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
    // vTaskDelay(2000);
  }
}

void Task2(void *pvParameters)
{
  while (1)
  {
    vTaskDelay(15);
    tcpController.sendServoAngle();
  }
}

void Task3(void *pvParameters)
{
  while (1)
  {
    vTaskDelay(15);
    motionController.servoServe();
  }
}

void Task4(void *pvParameters)
{
  while (1)
  {
    vTaskDelay(10);
    motionController.autoAvoidanceWorker();
  }
}

void Task5(void *pvParameters)
{
  while (1)
  {
    if (xSemaphoreTake(motionController.obstacleTriggerSemaphore, portMAX_DELAY) == pdTRUE)
      tcpController.sendTriggerObstacle();
  }
}