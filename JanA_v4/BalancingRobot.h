#ifndef _balancingrobot_h_
#define _balancingrobot_h_

#include <stdint.h> // Needed for uint8_t

bool sendData;
bool sendPIDValues;

#define PWM_FREQUENCY 20000 // The motor driver can handle a pwm frequency up to 20kHz
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used for the PS3 Communication and motor functions */
int lastCommand; // This is used set a new targetPosition
enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  joystick,
};

/* These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation 
 I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/ */
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/* Left motor */
// see Arduino PortManipulation http://www.arduino.cc/en/Reference/PortManipulation
#define leftPort PORTD // PORTD maps to Arduino digital pins 0 to 7
#define leftPortDirection DDRD
#define leftA PIND7 // pin 7 - I2
#define leftB PIND6 // pin 6 - I1

#define leftPwmPortDirection DDRB
#define leftPWM PINB1 // PB1 - pin 9 (OC1A) - EA

/* Right motor */
#define rightPort PORTB // PORTC maps to Arduino analog pins 0 to 5. Pins 6 & 7 are only accessible on the Arduino Mini
#define rightPortDirection DDRB
#define rightA PINB3 // pin 11 - I4
#define rightB PINB4 // pin 12 - I3

#define rightPwmPortDirection DDRB
#define rightPWM PINB2 // PB2 - pin 10 (OC1B) - EB

/* Encoders */
#define leftEncoder1 PIND2
#define leftEncoder2 PINC0
#define rightEncoder1 PIND3
#define rightEncoder2 PINC1

volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;

double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU */
int16_t accY;
int16_t accZ;
int16_t gyroX;

uint8_t i2cBuffer[14]; // Buffer for I2C data

// Results
double accAngle;
double gyroRate;
double gyroAngle;
double pitch;

/* PID variables */
double Kp = 6.2;
double Ki = 0.0;
double Kd = 10;
double targetAngle = 180.8;

double lastError; // Store last angle error
double integratedError; // Store integrated error
double error;
double pTerm, iTerm, dTerm;
double PIDValue, PIDLeft, PIDRight;

/* Used for timing */
unsigned long timer;
uint32_t dataTimer; // This is used so it doesn't send data to often

#define STD_LOOP_TIME 10000 // Fixed time loop of 10 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime;

/* Direction set by the controllers or SPP library */
bool steerForward;
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRight;

bool stopped; // This is used to set new target position after breaking

bool layingDown = true; // Use to indicate if the robot is laying down

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

// Data send via SPP
double sppData1 = 0;
double sppData2 = 0;

uint8_t loopCounter = 0; // Used to update wheel velocity
long wheelPosition;
long lastWheelPosition;
long wheelVelocity;
long targetPosition;

const uint16_t zoneA = 12000;
const uint16_t zoneB = 6000;
const uint16_t zoneC = 1500;
const double positionScaleA = 900; // One resolution is 928 pulses per encoder
const double positionScaleB = 1200;
const double positionScaleC = 1500;
const double positionScaleD = 750;
const double velocityScaleMove = 100;
const double velocityScaleStop = 90;
const double velocityScaleTurning = 100;

#endif
