void PID(double restAngle, double offset, double dt) {  
  /* Update PID values */
  error = (restAngle - pitch);
  pTerm = cfg.P * error;
  integratedError += error*dt;
  integratedError = constrain(integratedError, -1.0, 1.0); // Limit the integrated error
  iTerm = (cfg.I*100.0) * integratedError;
  dTerm = (cfg.D/100.0) * (error - lastError)/dt;
  lastError = error;
  PIDValue = pTerm + iTerm + dTerm;
  
  PIDLeft = PIDValue;
  PIDRight = PIDValue;

  // for debug
  /*Serial.print("dt ");Serial.print(dt);Serial.print("\t");
  Serial.print("restAngle ");Serial.print(restAngle);Serial.print("\t");
  Serial.print("pitch ");Serial.print(pitch);Serial.print("\t");
  Serial.print("error ");Serial.print(error);Serial.print("\t");
  Serial.print("pTerm ");Serial.print(pTerm);Serial.print("\t");
  Serial.print("integratedError ");Serial.print(integratedError);Serial.print("\t");
  Serial.print("iTerm ");Serial.print(iTerm);Serial.print("\t");
  Serial.print("dTerm ");Serial.print(dTerm);Serial.print("\t");
  Serial.print("PIDValue ");Serial.print(PIDValue);Serial.print("\t");
  Serial.println("");
  */

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
