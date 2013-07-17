//001

#ifndef _juna_h_
#define _juna_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

bool sendData;
bool sendPIDValues;

/* pwm */
const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a pwm frequency up to 20kHz
const uint8_t PWM_PRESCALER = 1; // use no prescaling
const uint16_t PWMVALUE = (F_CPU/PWM_FREQUENCY/2/PWM_PRESCALER)-1; // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used to make commands more readable */
uint8_t lastCommand; // This is used set a new targetPosition
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

bool ledState; // Last state of the built in LED

// This struct will store all the configuration values
typedef struct {
  // PID variables
  double P;
  double I;
  double D;
  
  double targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  
  double Qangle;
  double Qbias;
  double Rmeasure;
} cfg_t;

extern cfg_t cfg;

double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
int16_t accY;
int16_t accZ;
int16_t gyroX;

uint8_t i2cBuffer[14]; // Buffer for I2C data

// Results
double accAngle;
double gyroRate;
double gyroAngle;
double pitch;

double lastError; // Store last angle error
double integratedError; // Store integrated error

double error;
double pTerm, iTerm, dTerm;
double PIDValue, PIDLeft, PIDRight;

/* Used for timing */
uint32_t kalmanTimer; // Timer used for the Kalman filter
uint32_t pidTimer; // Timer used for the PID loop
uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
uint32_t dataTimer; // This is used so it doesn't send data to often
uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request

/* Direction set by the controllers or SPP library */
bool steerForward;
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRight;

bool stopped; // This is used to set a new target position after braking

bool layingDown = true; // Use to indicate if the robot is laying down

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

// Data send via SPP
double sppData1 = 0;
double sppData2 = 0;

int32_t wheelPosition; // Wheel position based on encoder readings
int32_t lastWheelPosition; // Used to calculate the wheel velocity
int32_t wheelVelocity; // Wheel velocity based on encoder readings
int32_t targetPosition; // The encoder position the robot should be at

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
