#include "BalancingRobot.h"
#include <Wire.h>
#include "Kalman.h" // Kalman filter library see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code

#include <SoftwareSerial.h>
SoftwareSerial SerialBT(2, 3); // TX, RX

void setup() {
  SerialBT.begin(57600);
  
  /* Setup motor pins to output */
  sbi(leftPwmPortDirection,leftPWM);
  sbi(leftPortDirection,leftA);
  sbi(leftPortDirection,leftB);
  sbi(rightPwmPortDirection,rightPWM);
  sbi(rightPortDirection,rightA);
  sbi(rightPortDirection,rightB);  

  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 */
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
  
  setPWM(leftPWM,0); // Turn off pwm on both pins
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
  loopStartTime = micros();
  timer = loopStartTime;
  dataTimer = millis();
}

void loop() {
  /* Update all the values */  
  while (i2cRead(0x3D,i2cBuffer,8));
  accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  //
  gyroRate = (double)gyroX/131.0; // Convert to deg/s
  gyroAngle += gyroRate*((double)(micros()-timer)/1000000.0); // Gyro angle is only used for debugging
  if (gyroAngle < 0 || gyroAngle > 360)
    gyroAngle = pitch; // Reset the gyro angle when it has drifted too much

  pitch = kalman.getAngle(accAngle, gyroRate, (double)(micros()-timer)/1000000.0); // Calculate the angle using a Kalman filter
  timer = micros();  

  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if((layingDown && (pitch < 170 || pitch > 190)) || (!layingDown && (pitch < 135 || pitch > 225))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } 
  else {
    layingDown = false; // It's no longer laying down
    PID(targetAngle,targetOffset,turningOffset);        
  }

  /* Use a time fixed loop */
  lastLoopUsefulTime = micros() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    while((micros() - loopStartTime) < STD_LOOP_TIME);
  }
  loopStartTime = micros();    
}

void PID(double restAngle, double offset, double turning) {
  /* Steer robot */
  if (steerForward) {
    restAngle -= offset;
  } 
  else if (steerBackward) {
    restAngle += offset;
  }
  /* Brake */
  else if (steerStop) {
  }
  
  /* Update PID values */
  error = (restAngle - pitch);
  pTerm = Kp * error;
  integratedError += error;
  integratedError = constrain(integratedError, -40.0, 40.0); // Limit the integrated error
  iTerm = Ki * integratedError;

  double derr = error - lastError;
  if(derr > -0.5 && derr < 0.5) derr = 0;
  dTerm = Kd * derr;
  
  PIDValue = pTerm + iTerm + dTerm;
  
  /* Read the SPP connection */
  readSPP();
  sendBluetoothData();
  
  lastError = error;

  /* Steer robot sideways */
  if (steerLeft) {
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  PIDRight *= 0.90; // compensate for difference in the motors

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, PIDLeft * -1);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, PIDRight * -1);
}
void sendBluetoothData() {
  if (millis() - dataTimer > 50) {  // Only send data every 50ms
    if (sendPIDValues) {
      sendPIDValues = false;
      dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
      SerialBT.print("P,");
      SerialBT.print(Kp);
      SerialBT.print(',');
      SerialBT.print(Ki);
      SerialBT.print(',');
      SerialBT.print(Kd);
      SerialBT.print(',');
      SerialBT.println(targetAngle);
    } else if (sendData) {
      dataTimer = millis();
      
      SerialBT.print("V,");
      SerialBT.print(accAngle);
      SerialBT.print(',');
      SerialBT.print(gyroAngle);
      SerialBT.print(',');
      SerialBT.print(pitch);
      SerialBT.print(',');
      SerialBT.print(error);
      SerialBT.print(',');
      SerialBT.print((error - lastError));
      SerialBT.print(',');
      SerialBT.print(pTerm);
      SerialBT.print(',');
      SerialBT.print(iTerm);
      SerialBT.print(',');
      SerialBT.print(dTerm);
      SerialBT.print(',');
      SerialBT.println(PIDValue);
    }
  }
}
void readSPP() {
    if (SerialBT.available()) {
    char input[30];
    uint8_t i = 0;
    while (1) {
      input[i] = SerialBT.read();   
      if (input[i] == -1) // Error while reading the string
        return;
      if (input[i] == ';') // Keep reading until it reads a semicolon
        break;
      i++;
      if (i >= sizeof(input)/sizeof(input[0])) // String is too long
        return;
    }
    
    if (input[0] == 'A') { // Abort
      stopAndReset();
      while (SerialBT.read() != 'C'); // Wait until continue is send
    }
    
    /* For sending PID and IMU values */
    else if (input[0] == 'G') { // The Processing/Android application sends when it needs the PID, settings or info
      if (input[1] == 'P') // Get PID Values
        sendPIDValues = true;
    }
    
    else if (input[0] == 'S') { // Set different values     
      /* Set PID and target angle */
      if (input[1] == 'P') {
        strtok(input, ","); // Ignore 'P'
        Kp = atof(strtok(NULL, ";"));
      } else if (input[1] == 'I') {
        strtok(input, ","); // Ignore 'I'
        Ki = atof(strtok(NULL, ";"));
      } else if (input[1] == 'D') {
        strtok(input, ","); // Ignore 'D'
        Kd = atof(strtok(NULL, ";"));
      } else if (input[1] == 'T') { // Target Angle
        strtok(input, ","); // Ignore 'T'
        targetAngle = atof(strtok(NULL, ";"));
      }
    }

    else if (input[0] == 'I') { // IMU trasmitting states
      if (input[1] == 'B') // Begin sending IMU values
        sendData = true; // Send output to Processing/Android application
      else if (input[1] == 'S') // Stop sending IMU values
        sendData = false; // Stop sending output to Processing/Android application
    }
    
    else if (input[0] == 'C') { // Commands
        if (input[1] == 'S') // Stop
          steer(stop);
        else if (input[1] == 'J') { // Joystick
          strtok(input, ","); // Ignore 'J'
          sppData1 = atof(strtok(NULL, ",")); // x-axis
          sppData2 = atof(strtok(NULL, ";")); // y-axis
          steer(joystick);
        }    
      }
    
  }
}
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);  
  lastError = 0;
  integratedError = 0;
}

void moveMotor(Command motor, Command direction, double speedRaw) { // Speed is a value in percentage 0-100%
  if(speedRaw > 100)
    speedRaw = 100;
  int speed = speedRaw*((double)PWMVALUE)/100; // Scale from 100 to PWMVALUE
  if (motor == left) {
    setPWM(leftPWM,speed); // Left motor pwm
    if (direction == forward) {
      cbi(leftPort,leftA);
      sbi(leftPort,leftB);
    } 
    else if (direction == backward) {
      sbi(leftPort,leftA);
      cbi(leftPort,leftB);
    }
  } 
  else if (motor == right) {
    setPWM(rightPWM,speed); // Right motor pwm
    if (direction == forward) {
      sbi(rightPort,rightA);
      cbi(rightPort,rightB);
    } 
    else if (direction == backward) {
      cbi(rightPort,rightA);
      sbi(rightPort,rightB);
    }
  }
}
void stopMotor(Command motor) {  
  if (motor == left) {
    setPWM(leftPWM,PWMVALUE); // Set high
    sbi(leftPort,leftA);
    sbi(leftPort,leftB);
  } 
  else if (motor == right) {
    setPWM(rightPWM,PWMVALUE); // Set high
    sbi(rightPort,rightA);
    sbi(rightPort,rightB);
  }
}

void setPWM(uint8_t pin, int dutyCycle) { // dutyCycle is a value between 0-ICR
  if(pin == leftPWM) {
    OCR1AH = (dutyCycle >> 8); 
    OCR1AL = (dutyCycle & 0xFF);
  } else if (pin == rightPWM) {
    OCR1BH = (dutyCycle >> 8);
    OCR1BL = (dutyCycle & 0xFF);    
  }
}



const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress,&data,1,sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes,(uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if(inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));              
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if(output > outputMax)
    output = outputMax;
  else if(output < outputMin)
    output = outputMin;
  return output;
}

void steer(Command command) {
  // Set all to false
  steerForward = false;
  steerBackward = false;
  steerStop = false;
  steerLeft = false;
  steerRight = false;
  
  if (command == joystick) {
    if (sppData2 > 0) {
      targetOffset = scale(sppData2,0,1,0,7);
      steerForward = true;
    } else if (sppData2 < 0) {
      targetOffset = scale(sppData2,0,-1,0,7);
      steerBackward = true;
    }
    if (sppData1 > 0) {
      turningOffset = scale(sppData1,0,1,0,20);
      steerRight = true;
    } else if (sppData1 < 0) {
      turningOffset = scale(sppData1,0,-1,0,20);
      steerLeft = true;
    }
  }
  
  if (command == stop) {
    steerStop = true;
  }
}
