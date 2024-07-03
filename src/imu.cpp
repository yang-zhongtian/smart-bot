#include "imu.h"
#include "constants.h"

void IMU::setup(uint8_t pin)
{
    uint8_t mpuIntStatus; // Holds actual interrupt status byte from MPU
    uint8_t devStatus;    // Return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;  // Expected DMP packet size (default is 42 bytes)

    // Initialize i2c
    Wire.begin();
    Wire.setClock(400000);

    // Initialize device
    mpu.initialize();

    // Verify connection
    if (!mpu.testConnection())
    {
        return;
    }

    pinMode(pin, INPUT);

    // Load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // Turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), [] {}, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void IMU::update()
{
    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector

    // MPU control/status vars
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    // read temperature
    temperature = mpu.getTemperature();
}

float IMU::getYaw()
{
    return ypr[0] * 180 / M_PI;
}

float IMU::getPitch()
{
    return ypr[1] * 180 / M_PI;
}

float IMU::getRoll()
{
    return ypr[2] * 180 / M_PI;
}

int16_t IMU::getTemperature()
{
    return temperature;
}
