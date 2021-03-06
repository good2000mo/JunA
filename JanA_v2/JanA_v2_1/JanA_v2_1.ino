/* Use this to enable and disable the different controllers */
#define ENABLE_SPP

#include "JunA.h"
#include <Wire.h>

#include "Kalman.h"

#ifdef ENABLE_SPP
#include <SoftwareSerial.h>
#endif

// Create the Kalman library instance
Kalman kalman;

#ifdef ENABLE_SPP
// BT
SoftwareSerial SerialBT(2, 3); // TX, RX
#endif

void setup()
{
  /* Initialize UART */
  Serial.begin(115200);
  
  /* Initialize BT Serial */
#ifdef ENABLE_SPP
  SerialBT.begin(57600);
#endif
  
  /* Read the PID values, target angle and other config */
  restoreConfigValues();
  
  /* Setup motor pins to output */
  // Set PB1 and PB2 as outputs
  // the code below is the same as pinMode(9, OUTPUT), pinMode(10, OUTPUT)
  // ex. DDRB |= _BV(1) | _BV(2);
  sbi(leftPwmPortDirection,leftPWM);
  sbi(leftPortDirection,leftA);
  sbi(leftPortDirection,leftB);
  sbi(rightPwmPortDirection,rightPWM);
  sbi(rightPortDirection,rightA);
  sbi(rightPortDirection,rightB);
  
  /* Set PWM frequency to 20kHz */
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1A = 0;
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);
  
  /* Enable PWM on pin 9 (OC1A) & pin 10 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcountin
  TCCR1A |= _BV(COM1A1);  // enable PWM on port B1 in non-inverted compare mode 2
  TCCR1A |= _BV(COM1B1);  // enable PWM on port B2 in non-inverted compare mode 2
  
  /* Turn off pwm on both pins */
  setPWM(leftPWM,0);
  setPWM(rightPWM,0);  
  
  /* Setup IMU */
  Wire.begin();
  i2cBuffer[0] = 19; // Set the sample rate to 400Hz - 8kHz/(19+1) = 400Hz
  i2cBuffer[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19,i2cBuffer,4,false)); // Write to all four registers at once
  while (i2cWrite(0x6B,0x09,true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode
  
  while (i2cRead(0x75,i2cBuffer,1));
  if (i2cBuffer[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1); // Halt
  }

  delay(100); // Wait for the sensor to get ready
  
  /* Set Kalman and gyro starting angle */
  while (i2cRead(0x3D,i2cBuffer,4));
  accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalman.setAngle(accAngle); // Set starting angle
  gyroAngle = accAngle;
  
  /* Setup timing */  
  kalmanTimer = micros();
  pidTimer = kalmanTimer;
  loopStartTime = kalmanTimer;
  dataTimer = millis();
}

void loop()
{
  /* Update all the values */  
  while (i2cRead(0x3D,i2cBuffer,8));
  accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
  if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
    pitch = accAngle;
    gyroAngle = accAngle;
    kalman.setAngle(accAngle);
  } else {
    gyroRate = (double)gyroX/131.0; // Convert to deg/s
    gyroAngle += gyroRate*((double)(micros()-kalmanTimer)/1000000.0); // Gyro angle is only used for debugging
    if (gyroAngle < 0 || gyroAngle > 360)
      gyroAngle = pitch; // Reset the gyro angle when it has drifted too much

    pitch = kalman.getAngle(accAngle, gyroRate, (double)(micros()-kalmanTimer)/1000000.0); // Calculate the angle using a Kalman filter
  }
  kalmanTimer = micros();
  
  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if ((layingDown && (pitch < cfg.targetAngle-10 || pitch > cfg.targetAngle+10)) || (!layingDown && (pitch < cfg.targetAngle-45 || pitch > cfg.targetAngle+45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  }
  else {
    layingDown = false; // It's no longer laying down
    PID(cfg.targetAngle,targetOffset,(double)(micros()-pidTimer)/1000000.0);
  }
  
  pidTimer = micros();
  
  /* Use a time fixed loop */
  //while((micros() - loopStartTime) < STD_LOOP_TIME);
  //loopStartTime = micros();
}


