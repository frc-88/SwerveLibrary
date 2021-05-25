package frc.team88.swerve.configuration.subconfig;

import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.configuration.exceptions.InvalidConfigValueException;
import frc.team88.swerve.configuration.exceptions.SwerveConfigException;
import frc.team88.swerve.data.NetworkTablePopulator;
import java.util.Objects;

/** Captures all of the configuration information about a position sensor. */
public class SensorTransmissionConfiguration implements NetworkTablePopulator {

  // Configuration values. See getters for documentation.
  private boolean inverted;
  private double ratio;
  private double offset;

  private transient boolean firstNetworkTableCall = true;

  /**
   * Constructs this configuration from a position sensor config.
   *
   * @param config The instantiated position sensor template.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  public SensorTransmissionConfiguration(Config config) {
    Objects.requireNonNull(config);
    this.inverted = Configuration.configCheckAndGet(config, "inverted", Boolean.class);
    this.ratio = Configuration.configCheckAndGetDouble(config, "ratio");
    if (this.ratio <= 0) {
      throw new InvalidConfigValueException("Ratio must be positive.");
    }
    this.offset = Configuration.configCheckAndGetDouble(config, "offset");
  }

  /**
   * Gets if the sensor is inverted.
   *
   * @return If the sensor is inverted.
   */
  public boolean isInverted() {
    return this.inverted;
  }

  /**
   * Gets if the sensor is inverted.
   *
   * @return If the sensor is inverted.
   */
  public boolean getInverted() {
    return this.inverted;
  }

  /**
   * Gets the sensor's gear ratio.
   *
   * @return The sensors gear ratio, which multiplied by the output value gives the input value.
   */
  public double getRatio() {
    return this.ratio;
  }

  /**
   * Gets the sensor's offset.
   *
   * @return The offset of the sensor, which is subtracted from the position after applying the gear
   *     ratio and inversion.
   */
  public double getOffset() {
    return this.offset;
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    if (this.firstNetworkTableCall) {
      this.firstNetworkTableCall = false;
      table.getEntry("inverted").setBoolean(this.inverted);
      table.getEntry("ratio").setDouble(this.ratio);
      table.getEntry("offset").setDouble(this.offset);
    } else {
      this.inverted = table.getEntry("inverted").getBoolean(this.inverted);
      this.ratio = table.getEntry("ratio").getDouble(this.ratio);
      this.offset = table.getEntry("offset").getDouble(this.offset);
    }
  }
}
