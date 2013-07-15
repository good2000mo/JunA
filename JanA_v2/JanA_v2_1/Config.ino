cfg_t cfg;

void updateConfig() {
  kalman.setQangle(cfg.Qangle);
  kalman.setQbias(cfg.Qbias);
  kalman.setRmeasure(cfg.Rmeasure);
}

void restoreConfigValues() {
  cfg.P = 4.2;
  cfg.I = 0.0;
  cfg.D = 10;

  cfg.targetAngle = 182;
  
  cfg.Qangle = 0.001;
  cfg.Qbias = 0.003;
  cfg.Rmeasure = 0.03;

  updateConfig();
}
