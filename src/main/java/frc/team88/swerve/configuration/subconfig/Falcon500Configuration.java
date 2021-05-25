package frc.team88.swerve.configuration.subconfig;

import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.configuration.exceptions.InvalidConfigValueException;
import frc.team88.swerve.configuration.exceptions.SwerveConfigException;
import frc.team88.swerve.data.NetworkTablePopulator;
import java.util.Objects;

/** Captures all of the configuration information about a Falcon 500. */
public class Falcon500Configuration implements NetworkTablePopulator {

  // Configuration values. See getters for documentation.
  private boolean inverted;
  private double maxSpeed;

  private transient boolean firstNetworkTableCall = true;

  /**
   * Constructs this configuration from a falcon 500 config.
   *
   * @param config The instantiated falcon 500 template.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  public Falcon500Configuration(Config config) {
    Objects.requireNonNull(config);
    this.inverted = Configuration.configCheckAndGet(config, "inverted", Boolean.class);
    this.maxSpeed = Configuration.configCheckAndGetDouble(config, "max-speed-rps");
    if (this.maxSpeed <= 0) {
      throw new InvalidConfigValueException("Max speed must be positive.");
    }
  }

  /**
   * Gets if the motor is inverted.
   *
   * @return If the motor is inverted.
   */
  public boolean isInverted() {
    return this.inverted;
  }

  /**
   * Gets if the motor is inverted.
   *
   * @return If the motor is inverted.
   */
  public boolean getInverted() {
    return this.inverted;
  }

  /**
   * Gets the max speed of the motor.
   *
   * @return The measured max speed of the motor, including inefficiencies.
   */
  public double getMaxSpeed() {
    return this.maxSpeed;
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    if (this.firstNetworkTableCall) {
      this.firstNetworkTableCall = false;
      table.getEntry("inverted").setBoolean(this.inverted);
      table.getEntry("maxSpeed").setDouble(this.maxSpeed);
    } else {
      this.inverted = table.getEntry("inverted").getBoolean(this.inverted);
      this.maxSpeed = table.getEntry("maxSpeed").getDouble(this.maxSpeed);
    }
  }
}
