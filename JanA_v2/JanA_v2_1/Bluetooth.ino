#ifdef ENABLE_SPP
void sendBluetoothData() {
  if (millis() - dataTimer > 50) {  // Only send data every 50ms
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
      SerialBT.print(error);
      SerialBT.print(',');
      SerialBT.print(lastError);
      SerialBT.print(',');
      SerialBT.print(pTerm);
      SerialBT.print(',');
      SerialBT.print(integratedError);
      SerialBT.print(',');
      SerialBT.print(dTerm);
      SerialBT.print(',');
      SerialBT.println(PIDValue);
    }
  }
}

void readSPPData() {
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
      if (input[1] == 'R') {
        restoreConfigValues(); // Restore the default PID values and target angle
        sendPIDValues = true;
      }         
    }
  }
}
#endif // ENABLE_SPP

#if defined(ENABLE_SPP)
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
#endif // defined(ENABLE_SPP)

