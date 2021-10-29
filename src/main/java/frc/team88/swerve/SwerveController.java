package frc.team88.swerve;

import com.ctre.phoenix.CANifier;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.data.DataManager;
import frc.team88.swerve.gyro.SwerveGyro;
import frc.team88.swerve.module.util.ModuleDoubleSupplier;
import frc.team88.swerve.motion.SwerveChassis;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.tuning.TuningManager;
import java.util.Map;
import java.util.Objects;

/** The high-level API for controlling a swerve drive with this library. */
public class SwerveController {

  private final Configuration config;
  private final SwerveChassis chassis;
  private final TuningManager tuningManager;
  private final DataManager dataManager;

  /**
   * Constructs the SwerveController using the given toml config.
   *
   * @param configPath The file path of the toml config. It can be a relative path inside of the
   *     deploy directory or an absolute path.
   */
  public SwerveController(final String configPath) {
    Objects.requireNonNull(configPath);
    this.config = new Configuration(configPath);
    this.chassis = new SwerveChassis(this.config);
    this.tuningManager = new TuningManager(this.config);
    this.dataManager = new DataManager(this.config, this.chassis, this.tuningManager);
  }

  /**
   * Constructs the SwerveController using the given toml config.
   *
   * @param configPath The file path of the toml config. It can be a relative path inside of the
   *     deploy directory or an absolute path.
   * @param gyro The gyro to use instead of instantiating one from the config.
   */
  public SwerveController(final String configPath, SwerveGyro gyro) {
    Objects.requireNonNull(configPath);
    this.config = new Configuration(configPath, gyro);
    this.chassis = new SwerveChassis(this.config);
    this.tuningManager = new TuningManager(this.config);
    this.dataManager = new DataManager(this.config, this.chassis, this.tuningManager);
  }

  /**
   * Updates all controllers, data loggers, and anything else associated with this SwerveController.
   */
  public void update() {
    this.tuningManager.update();
    if (!this.tuningManager.isEnabled()) {
      this.chassis.update();
    }
    this.dataManager.update();
  }

  /**
   * Sets the translation and rotation velocity of the robot.
   *
   * @param velocityState Object containing the target velocity state
   */
  public void setVelocity(VelocityState velocityState) {
    this.chassis.setTargetState(velocityState);
    this.chassis.holdAzimuths(false);
  }

  /**
   * Sets the translation and rotation velocity of the robot.
   *
   * @param translationDirection The direction for the robot to move, in degrees counterclockwise
   *     from forwards.
   * @param translationSpeed The speed for the robot to move, in feet per second.
   * @param rotationVelocity The velocity for the robot to spin about the the center of rotation,
   *     independent of translation, in degrees per second.
   * @param fieldCentric If true, the given translation direction will be interpretted relative to
   *     the gyro's zero point, which should be consistently facing away from the driver. If false,
   *     it will be interpretted relative to the front of the robot.
   */
  public void setVelocity(
      double translationDirection,
      double translationSpeed,
      double rotationVelocity,
      boolean fieldCentric) {
    VelocityState velocityState = this.getTargetVelocity();
    velocityState =
        velocityState
            .changeTranslationDirection(translationDirection)
            .changeTranslationSpeed(translationSpeed)
            .changeRotationVelocity(rotationVelocity)
            .changeIsFieldCentric(fieldCentric);
    this.chassis.setTargetState(velocityState);
    this.chassis.holdAzimuths(false);
  }

  /**
   * Sets the translation speed and rotation velocity of the robot. Does not modify translation
   * direction or center of rotation.
   *
   * @param translationSpeed The speed for the robot to move, in feet per second.
   * @param rotationVelocity The velocity for the robot to spin about the the center of rotation,
   *     independent of translation, in degrees per second.
   */
  public void setVelocity(double translationSpeed, double rotationVelocity) {
    VelocityState velocityState = this.getTargetVelocity();
    velocityState =
        velocityState
            .changeTranslationSpeed(translationSpeed)
            .changeRotationVelocity(rotationVelocity);
    this.chassis.setTargetState(velocityState);
    this.chassis.holdAzimuths(false);
  }

  /**
   * Sets the translation direction of the robot.
   *
   * @param translationDirection The direction for the robot to move, in degrees counterclockwise
   *     from forwards.
   * @param fieldCentric If true, the given translation direction will be interpretted relative to
   *     the gyro's zero point, which should be consistently facing away from the driver. If false,
   *     it will be interpretted relative to the front of the robot.
   */
  public void setTranslationDirection(double translationDirection, boolean fieldCentric) {
    VelocityState velocityState = this.getTargetVelocity();
    velocityState =
        velocityState
            .changeTranslationDirection(translationDirection)
            .changeIsFieldCentric(fieldCentric);
    this.chassis.setTargetState(velocityState);
    this.chassis.holdAzimuths(false);
  }

  /**
   * Sets the point that the robot will rotate around, relative to the origin used to define the
   * location of the swerve modules in the config. It does not need to be within the robot's frame
   * perimeter. Positive x corresponds to robot forwards, positive y corresponds to robot left.
   *
   * @param x The x component, in feet.
   * @param y The y component, in feet.
   */
  public void setCenterOfRotation(double x, double y) {
    VelocityState velocityState = this.getTargetVelocity();
    velocityState = velocityState.changeCenterOfRotation(x, y);
    this.chassis.setTargetState(velocityState);
  }

  /** Sets the robot to stop moving and hold it's current swerve module positions. */
  public void holdDirection() {
    VelocityState velocityState = this.getTargetVelocity();
    velocityState = velocityState.changeTranslationSpeed(0).changeRotationVelocity(0);
    this.chassis.setTargetState(velocityState);
    this.chassis.holdAzimuths(true);
  }

  /**
   * Offsets the yaw readings from the gyro so that the robot's current heading will be set to the
   * given yaw.
   *
   * @param yaw The yaw to offset the gyro to.
   */
  public void setGyroYaw(double yaw) {
    this.getGyro().calibrateYaw(yaw);
  }

  /**
   * Gets the maximum translation speed if the drive is doing nothing else.
   *
   * @return The maximum translation speed, in feet per second.
   */
  public double getMaxTranslationSpeed() {
    return this.chassis.getMaxTranslationSpeed();
  }

  /**
   * Gets the maximum rotation speed if the drive is doing nothing else.
   *
   * @return The maximum rotation speed, in feet per second.
   */
  public double getMaxRotationSpeed() {
    return this.chassis.getMaxTranslationSpeed();
  }

  /**
   * Sets all motors on the swerve drive to coast. This is not recommended during driving, and is
   * only provided the for ease of moving the modules while disabled.
   */
  public void setCoast() {
    this.chassis.setCoast();
  }

  /** Sets all motors on the swerve drive to brake. This is the default behavior. */
  public void setBrake() {
    this.chassis.setBrake();
  }

  /**
   * Sets strategy for azimuth flipping for all modules
   *
   * @param supplier A lambda function of type ModuleDoubleSupplier. Takes a module, returns bias as
   *     double
   */
  public void setAzimuthWrapBiasStrategy(ModuleDoubleSupplier supplier) {
    this.chassis.setAzimuthWrapBiasStrategy(supplier);
  }

  /**
   * Sets strategy for azimuth flipping for a single module
   *
   * @param moduleIndex index of module to set
   * @param supplier A lambda function of type ModuleDoubleSupplier. Takes a module, returns bias as
   *     double
   */
  public void setAzimuthWrapBiasStrategy(int moduleIndex, ModuleDoubleSupplier supplier) {
    this.chassis.setAzimuthWrapBiasStrategy(moduleIndex, supplier);
  }

  /**
   * Gets the velocity state that this controller is trying to achieve.
   *
   * @return The target velocity state.
   */
  public VelocityState getTargetVelocity() {
    return this.chassis.getTargetState();
  }

  /**
   * Gets the velocity state that is being commanded at this moment based on the various constraints
   * around how quickly the robot can change it's motion.
   *
   * @return The commanded velocity state after constraints have been applied.
   */
  public VelocityState getConstrainedVelocity() {
    return this.chassis.getConstrainedCommandState();
  }

  /**
   * Gets the odometry readings, which estimate the robot's position and velocity based on measured
   * wheel speed and angle.
   *
   * @return The current odometry state.
   */
  public OdomState getOdometry() {
    return this.chassis.getOdomState();
  }

  /**
   * Gets the gyro being used by this swerve controller.
   *
   * @return The gyro object.
   */
  public SwerveGyro getGyro() {
    return this.config.getGyro();
  }

  /**
   * Gets the CANifiers used by this swerve for sensing, if any.
   *
   * @return A mapping from can IDs to Canifiers with that ID.
   */
  public Map<Integer, CANifier> getCanifiers() {
    return this.config.getCanifiers();
  }

  /** Enables publishing swerve data to NetworkTables. */
  public void enableNetworkTablesPublishing() {
    this.dataManager.setEnableNetworkTablesPublishing(true);
  }

  /** Disables pushing swerve data to NetworkTables. */
  public void disableNetworkTablesPublishing() {
    this.dataManager.setEnableNetworkTablesPublishing(false);
  }
}
