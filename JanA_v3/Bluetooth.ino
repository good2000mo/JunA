//001

#ifdef ENABLE_SPP
void sendBluetoothData() {
  if (SerialBT.isListening() && (millis() - dataTimer > 50)) {  // Only send data every 50ms
    if (sendPIDValues) {
      sendPIDValues = false;
      dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
      SerialBT.print("P,");
      SerialBT.print(cfg.P);
      SerialBT.print(',');
      SerialBT.print(cfg.I);
      SerialBT.print(',');
      SerialBT.print(cfg.D);
      SerialBT.print(',');
      SerialBT.println(cfg.targetAngle);
    } else if (sendData) {
      dataTimer = millis();
      
      SerialBT.print("V,");
      SerialBT.print(accAngle);
      SerialBT.print(',');
      SerialBT.print(gyroAngle);
      SerialBT.print(',');
      SerialBT.print(pitch);
      SerialBT.print(',');
      SerialBT.print(steerForward);
      SerialBT.print(',');
      SerialBT.print(steerStop);
      SerialBT.print(',');
      SerialBT.print(pTerm);
      SerialBT.print(',');
      SerialBT.print(integratedError);
      SerialBT.print(',');
      SerialBT.print(dTerm);
      SerialBT.print(',');
      SerialBT.println(PIDValue);

/*
      Serial.print(accAngle);
      Serial.print("\t");
      Serial.print(gyroAngle);
      Serial.print("\t");
      Serial.print(pitch);
      Serial.print("\t");
      Serial.print(leftCounter);
      Serial.print("\t");
      Serial.print(rightCounter);
      Serial.print("\t");
      Serial.print(wheelPosition);
      Serial.print("\t");
      Serial.print(wheelVelocity);
      Serial.println("\t");
      */
    }
  }
}

void readSPPData() {
  if (SerialBT.isListening() && SerialBT.available()) {
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
        cfg.P = atof(strtok(NULL, ";"));
      } else if (input[1] == 'I') {
        strtok(input, ","); // Ignore 'I'
        cfg.I = atof(strtok(NULL, ";"));
      } else if (input[1] == 'D') {
        strtok(input, ","); // Ignore 'D'
        cfg.D = atof(strtok(NULL, ";"));
      } else if (input[1] == 'T') { // Target Angle
        strtok(input, ","); // Ignore 'T'
        cfg.targetAngle = atof(strtok(NULL, ";"));
      }
      updateConfig();
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
      if (input[1] == 'R') {
        restoreConfigValues(); // Restore the default PID values and target angle
        sendPIDValues = true;
      }         
    }
  }
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
      targetOffset = scale(sppData2,0,1,0,cfg.controlAngleLimit);
      steerForward = true;
    } else if (sppData2 < 0) {
      targetOffset = scale(sppData2,0,-1,0,cfg.controlAngleLimit);
      steerBackward = true;
    }
    if (sppData1 > 0) {
      turningOffset = scale(sppData1,0,1,0,cfg.turningLimit);
      steerRight = true;
    } else if (sppData1 < 0) {
      turningOffset = scale(sppData1,0,-1,0,cfg.turningLimit);
      steerLeft = true;
    }
  }

  if (command == stop) {
    steerStop = true;
    if (lastCommand != stop) { // Set new stop position, 如果之前正在前進或後退，那這個stop就會重設targetposition，為了backspot
      targetPosition = wheelPosition;
      stopped = false; // non braking
    }
  }
  lastCommand = command;
}

double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if (inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}
#endif // ENABLE_SPP
