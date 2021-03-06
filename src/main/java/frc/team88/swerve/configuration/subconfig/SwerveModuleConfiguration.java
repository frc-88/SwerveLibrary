package frc.team88.swerve.configuration.subconfig;

import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.configuration.exceptions.ConfigFieldNotFoundException;
import frc.team88.swerve.configuration.exceptions.InvalidConfigValueException;
import frc.team88.swerve.configuration.exceptions.SwerveConfigException;
import frc.team88.swerve.data.NetworkTablePopulator;
import frc.team88.swerve.util.Vector2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

/** Captures all of the configuration information about a SwerveModule. */
public class SwerveModuleConfiguration implements NetworkTablePopulator {

  // Configuration values. See getters for documentation.
  private Vector2D location;
  private final RealMatrix forwardMatrix;
  private final RealMatrix inverseMatrix;
  private double wheelDiameter;
  private final TrapezoidalControllerConfiguration azimuthControllerConfig;
  private final PIDConfiguration wheelControllerConfig;

  private transient boolean firstNetworkTableCall = true;

  /**
   * Constructs this configuration from an instantiated module template.
   *
   * @param config The instantiated module template.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  public SwerveModuleConfiguration(Config config) {
    Objects.requireNonNull(config);
    this.location =
        Vector2D.createCartesianCoordinates(
            Configuration.configCheckAndGetDouble(config, "location-inches.x") / 12.0,
            Configuration.configCheckAndGetDouble(config, "location-inches.y") / 12.0);

    // Convert 2D list to 2D array
    if (!config.contains("differential-matrix")) {
      throw new ConfigFieldNotFoundException(
          "The differential-matrix field was not found in a swerve module config.");
    }
    this.forwardMatrix =
        new Array2DRowRealMatrix(
            convertObjectListTo2DDoubleArray(config.get("differential-matrix")), true);
    this.inverseMatrix = MatrixUtils.inverse(this.forwardMatrix);

    this.wheelDiameter =
        Configuration.configCheckAndGetDouble(config, "wheel-diameter-inches") / 12.0;
    if (this.wheelDiameter <= 0) {
      throw new InvalidConfigValueException(
          String.format("Wheel diameter is %d, but it should be positive.", this.wheelDiameter));
    }
    this.azimuthControllerConfig =
        new TrapezoidalControllerConfiguration(
            Configuration.configCheckAndGet(config, "azimuth-controller", Config.class));
    this.wheelControllerConfig =
        new PIDConfiguration(
            Configuration.configCheckAndGet(config, "wheel-controller", Config.class));
  }

  /**
   * Gets the location of the swerve module.
   *
   * @return The location of the swerve module in feet.
   */
  public Vector2D getLocation() {
    return this.location;
  }

  /**
   * Gets the forward differential matrix.
   *
   * @return The 2x2 forward differential matrix for the module, which converts from inputs [motor0,
   *     motor1] to outputs [azimuthSpeed, wheelSpeed]. Unitless.
   */
  public RealMatrix getForwardMatrix() {
    return this.forwardMatrix;
  }

  /**
   * Gets the inverse differential matrix.
   *
   * @return The 2x2 inverse differential matrix for the module, which converts from outputs
   *     [azimuthSpeed, wheelSpeed] to inputs [motor0, motor1]. Unitless.
   */
  public RealMatrix getInverseMatrix() {
    return this.inverseMatrix;
  }

  /**
   * Gets the wheel size.
   *
   * @return The diameter of the wheel on the module, in feet.
   */
  public double getWheelDiameter() {
    return this.wheelDiameter;
  }

  /**
   * Gets the azimuth controller configuration.
   *
   * @return The configuration info for the azimuth position controller.
   */
  public TrapezoidalControllerConfiguration getAzimuthControllerConfig() {
    return this.azimuthControllerConfig;
  }

  /**
   * Gets the wheel controller configuration
   *
   * @return The configuration info for the wheel velocity contoller.
   */
  public PIDConfiguration getWheelControllerConfig() {
    return this.wheelControllerConfig;
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    this.wheelControllerConfig.populateNetworkTable(table.getSubTable("wheelController"));
    this.azimuthControllerConfig.populateNetworkTable(table.getSubTable("azimuthController"));
    if (this.firstNetworkTableCall) {
      this.firstNetworkTableCall = false;
      table.getEntry("wheelDiameter").setDouble(this.wheelDiameter);
      table.getEntry("locationX").setDouble(this.location.getX());
      table.getEntry("locationY").setDouble(this.location.getY());
    } else {
      this.wheelDiameter = table.getEntry("wheelDiameter").getDouble(this.wheelDiameter);
      this.location =
          Vector2D.createCartesianCoordinates(
              table.getEntry("locationX").getDouble(this.location.getX()),
              table.getEntry("locationY").getDouble(this.location.getY()));
    }
  }

  /**
   * Converts a 2D list of Doubles typed as Object into a 2D double array.
   *
   * @param list A 2x2 nested list of Objects which will be casted to Double objects.
   * @return A 2x2 double array.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private double[][] convertObjectListTo2DDoubleArray(List<?> list) {
    double arr[][] = new double[2][2];
    if (!(list instanceof List<?>)) {
      throw new InvalidConfigValueException("Differential matrix is not a list.");
    }
    List<Object> outerList = new ArrayList<Object>(list);
    if (outerList.size() != 2) {
      throw new InvalidConfigValueException("Differential matrix does not have height 2.");
    }

    for (int row = 0; row < 2; row++) {
      Object outerItem = outerList.get(row);
      if (!(outerItem instanceof List<?>)) {
        throw new InvalidConfigValueException("Differential matrix is not a list of lists.");
      }
      List<Object> innerList = new ArrayList<Object>((List<?>) outerItem);
      if (innerList.size() != 2) {
        throw new InvalidConfigValueException("Differential matrix does not have width 2.");
      }

      for (int col = 0; col < 2; col++) {
        Object innerItem = innerList.get(col);
        if (!(innerItem instanceof Number)) {
          throw new InvalidConfigValueException(
              "Differential matrix contains a non-number element.");
        }
        Number value = (Number) (innerItem);
        arr[row][col] = value.doubleValue();
      }
    }
    return arr;
  }
}
