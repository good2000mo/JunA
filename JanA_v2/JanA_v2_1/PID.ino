void PID(double restAngle, double offset, double dt) {
  /* Update PID values */
  error = (restAngle - pitch);
  pTerm = cfg.P * error;
  integratedError += error*dt;
  integratedError = constrain(integratedError, -30.0, 30.0); // Limit the integrated error
  iTerm = (cfg.I*100.0) * integratedError;
  dTerm = (cfg.D/100.0) * (error - lastError)/dt;
  PIDValue = pTerm + iTerm + dTerm;
  
  temp = 1/dt;
  
#ifdef ENABLE_SPP
  readSPPData();
  sendBluetoothData();
#endif

  lastError = error;
  
  PIDLeft = PIDValue;
  PIDRight = PIDValue;

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, -PIDLeft);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, -PIDRight);
}
