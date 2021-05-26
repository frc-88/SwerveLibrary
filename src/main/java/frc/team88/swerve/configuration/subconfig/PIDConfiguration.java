package frc.team88.swerve.configuration.subconfig;

import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.configuration.exceptions.InvalidConfigValueException;
import frc.team88.swerve.configuration.exceptions.SwerveConfigException;
import frc.team88.swerve.data.NetworkTablePopulator;
import java.util.Objects;

/** Contains PID values pulled from a config file. */
public class PIDConfiguration implements NetworkTablePopulator {

  // Configuration values. See getters for documentation.
  private double kP;
  private double kI;
  private double kD;
  private double kF;
  private double iZone;
  private double iMax;
  private double tolerance;

  private transient boolean firstNetworkTableCall = true;

  /**
   * Constructs from a raw configuration containing some of the appropriate fields.
   *
   * @param config The raw configuration.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  public PIDConfiguration(Config config) {
    Objects.requireNonNull(config);
    this.kP = getPIDConstant(config, "kP");
    this.kI = getPIDConstant(config, "kI");
    this.kD = getPIDConstant(config, "kD");
    this.kF = getPIDConstant(config, "kF");
    this.iZone = getPIDConstant(config, "i-zone");
    this.iMax = getPIDConstant(config, "i-max");
    this.tolerance = getPIDConstant(config, "tolerance");
  }

  /**
   * Gets the kP value.
   *
   * @return The proportional constant.
   */
  public double getKP() {
    return this.kP;
  }

  /**
   * Gets the kI value.
   *
   * @return The integral constant.
   */
  public double getKI() {
    return this.kI;
  }

  /**
   * Gets the kD value.
   *
   * @return The differential constant.
   */
  public double getKD() {
    return this.kD;
  }

  /**
   * Gets the kF value.
   *
   * @return The feedforward constant.
   */
  public double getKF() {
    return this.kF;
  }

  /**
   * Gets the iZone value.
   *
   * @return The max error for which integral will be accumulated.
   */
  public double getIZone() {
    return this.iZone;
  }

  /**
   * Gets the iMax value.
   *
   * @return The max accumulated integral value.
   */
  public double getIMax() {
    return this.iMax;
  }

  /**
   * Gets the tolerance value.
   *
   * @return The minimum error for the PID output to be non-zero.
   */
  public double getTolerance() {
    return this.tolerance;
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    if (this.firstNetworkTableCall) {
      this.firstNetworkTableCall = false;
      table.getEntry("kP").setDouble(this.kP);
      table.getEntry("kI").setDouble(this.kI);
      table.getEntry("kD").setDouble(this.kD);
      table.getEntry("kF").setDouble(this.kF);
      table.getEntry("iZone").setDouble(this.iZone);
      table.getEntry("iMax").setDouble(this.iMax);
      table.getEntry("tolerance").setDouble(this.tolerance);
    } else {
      this.kP = table.getEntry("kP").getDouble(this.kP);
      this.kI = table.getEntry("kI").getDouble(this.kI);
      this.kD = table.getEntry("kD").getDouble(this.kD);
      this.kF = table.getEntry("kF").getDouble(this.kF);
      this.iZone = table.getEntry("iZone").getDouble(this.iZone);
      this.iMax = table.getEntry("iMax").getDouble(this.iMax);
      this.tolerance = table.getEntry("tolerance").getDouble(this.tolerance);
    }
  }

  /**
   * If the field exists, check that it's a positive number then return it as a double. Otherwise,
   * return 0.
   *
   * @param config The config to get value from.
   * @param key The key of the field to get.
   * @return The value of the field, or 0.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private static double getPIDConstant(Config config, String key) {
    double value = Configuration.configCheckAndGetDoubleOrElse(config, key, 0.);
    if (value < 0) {
      throw new InvalidConfigValueException(
          String.format(
              "Field %s contains %f, but PID constant values must be non-negative.", key, value));
    }
    return value;
  }
}
