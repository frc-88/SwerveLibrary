package frc.team88.swerve.commandmux;

import frc.team88.swerve.configuration.CommandMuxConfiguration;
import frc.team88.swerve.util.RobotControllerWrapper;
import java.util.Objects;

public class CommandMuxEntry {
  // The configuration data for this mux entry.
  private final CommandMuxConfiguration config;

  // timestamp of the last active time
  private double lastActiveTime;

  // Threshold to check if the clock jumps forwards or backwards
  private final double kTimeJumpThreshold = 100.0;

  public CommandMuxEntry(CommandMuxConfiguration config) {
    this.config = Objects.requireNonNull(config);
  }
  private double currentTime() {
    return RobotControllerWrapper.getInstance().getFPGATime() * 1E-6;
  }
  public void activate()
  {
    lastActiveTime = currentTime();
  }

  public boolean isActive()
  {
    double dt = currentTime() - lastActiveTime;
    if (dt > kTimeJumpThreshold || dt <= 0.) {  // ignore cases where the clock jumps forward or backwards suddenly
      return false;
    }
    if (this.config.getTimeout() <= 0.0) {  // if timeout is zero, consider it always active
      return true;
    }
    return dt < this.config.getTimeout();
  }

  public int getPriority()
  {
    return this.config.getPriority();
  }

  public int getID()
  {
    return this.config.getID();
  }

  public String getTemplateName()
  {
    return this.config.getTemplateName();
  }
}
