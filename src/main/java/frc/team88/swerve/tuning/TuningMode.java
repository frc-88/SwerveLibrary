package frc.team88.swerve.tuning;

import frc.team88.swerve.data.NetworkTablePopulator;

/** Represents a tuning mode to be managed by the TuningManager. */
public interface TuningMode extends NetworkTablePopulator {

  /** Called when this mode transitions from disabled to enabled. */
  public void init();

  /** Called repeatedly while this mode is enabled. */
  public void run();
}
