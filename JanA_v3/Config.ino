//001

cfg_t cfg;

void updateConfig() {
  kalman.setQangle(cfg.Qangle);
  kalman.setQbias(cfg.Qbias);
  kalman.setRmeasure(cfg.Rmeasure);
}

void restoreConfigValues() {
  cfg.P = 4.2;
  cfg.I = 0.0;
  cfg.D = 8.0;

  cfg.targetAngle = 180.8;
  cfg.backToSpot = 1;
  cfg.controlAngleLimit = 6;
  cfg.turningLimit = 20;
  
  cfg.Qangle = 0.001;
  cfg.Qbias = 0.003;
  cfg.Rmeasure = 0.03;

  updateConfig();
}
