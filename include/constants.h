#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

#define INTERRUPT_PIN 16

#define MAX_DATA_COUNT 10
#define MAX_DATA_SIZE 256

/* Special flags */
const float KEEP_CURRENT = 255;

/* Servo pins */
const int SERVO_PINS[4][3] = {
    {32, 33, 25},
    {19, 18, 5},
    {13, 14, 27},
    {15, 2, 4},
};
const int SERVO_OFFSETS[4][3] = {
    {-48, -12, -40},
    {75, -22, -50},
    {40, 20, -85},
    {-65, 10, 50},
};

/* Supersonic sensor trigger */
const int SUPERSONIC_TRIG = 23;

/* Size of the robot */
const float LENGTH_A = 55;
const float LENGTH_B = 77.5;
const float LENGTH_C = 27.5;
const float LENGTH_SIDE = 71;
const float Z_ABSOLUTE = -28;

/* Constants for movement */
const float Z_DEFAULT = -50, Z_RAISE_HEAD = -30, Z_BOOT = Z_ABSOLUTE;
const float X_DEFAULT = 62, X_OFFSET = 0;
const float Y_START = 0, Y_STEP = 40;
const float Y_DEFAULT = X_DEFAULT;

/* Constants for speed */
const float SPOT_TURN_SPEED = 4;
const float LEG_MOVE_SPEED = 8;
const float BODY_MOVE_SPEED = 3;
const float STAND_SIT_SPEED = 1;

/* Constants for turn */
// temp length
const float __TEMP_A = sqrt(pow(2 * X_DEFAULT + LENGTH_SIDE, 2) + pow(Y_STEP, 2));
const float __TEMP_B = 2 * (Y_START + Y_STEP) + LENGTH_SIDE;
const float __TEMP_C = sqrt(pow(2 * X_DEFAULT + LENGTH_SIDE, 2) + pow(2 * Y_START + Y_STEP + LENGTH_SIDE, 2));
const float __TEMP_ALPHA = acos((pow(__TEMP_A, 2) + pow(__TEMP_B, 2) - pow(__TEMP_C, 2)) / 2 / __TEMP_A / __TEMP_B);
// site for turn
const float TURN_X1 = (__TEMP_A - LENGTH_SIDE) / 2;
const float TURN_Y1 = Y_START + Y_STEP / 2;
const float TURN_X0 = TURN_X1 - __TEMP_B * cos(__TEMP_ALPHA);
const float TURN_Y0 = __TEMP_B * sin(__TEMP_ALPHA) - TURN_Y1 - LENGTH_SIDE;

#endif