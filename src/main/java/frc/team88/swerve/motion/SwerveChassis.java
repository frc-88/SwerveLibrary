package frc.team88.swerve.motion;

import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.kinematics.ForwardKinematics;
import frc.team88.swerve.motion.kinematics.InverseKinematics;
import frc.team88.swerve.motion.state.ModuleState;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import java.util.Objects;
import java.util.stream.Stream;

/** Represents a complete swerve chassis, with high level operations for controlling it. */
public class SwerveChassis {

  // The config for this swerve drive.
  private Configuration config;

  // The unmodified commanded target state.
  private VelocityState targetState = new VelocityState(0, 0, 0, false);

  // The commanded velocity state that obeys all constraints.
  private VelocityState constrainedState = this.targetState;

  // The inverse kinematics controller for this chassis.
  private InverseKinematics inverseKinematics;

  // The forward kinematics controller for this chassis.
  private ForwardKinematics forwardKinematics;

  // A mode for holding wheel azimuths and setting speed to 0.
  private boolean holdMode = true;
  ;

  /**
   * Constructs the SwerveChassis from the config.
   *
   * @param config The config info for this swerve.
   */
  public SwerveChassis(Configuration config) {
    this.config = Objects.requireNonNull(config);

    this.inverseKinematics = new InverseKinematics(this.config.getModules());
    this.forwardKinematics = new ForwardKinematics(this.config.getModules());
  }

  /**
   * Sets the target velocity state.
   *
   * @param target The velocity state to set.
   */
  public void setTargetState(VelocityState target) {
    this.targetState = Objects.requireNonNull(target);
  }

  /**
   * Sets an individual module's motor velocity.
   * If you use this method. Don't call update()
   *
   * @param moduleIndex Index of the module
   * @param motorIndex Index of the motor within the module
   * @param motorVelocity motor's velocity to set, in rotations per second.
   */
  public void setModuleMotor(int moduleIndex, int motorIndex, double motorVelocity)
  {
    SwerveModule module = this.config.getModules()[moduleIndex];
    module.setModuleMotorVelocity(motorIndex, motorVelocity);
  }

  /**
   * Sets an individual module's state.
   * If you use this method. Don't call update()
   *
   * @param moduleIndex Index of the module
   * @param moduleState Commanded state of the module
   */
  public void setModuleState(int moduleIndex, ModuleState moduleState) {
    SwerveModule module = this.config.getModules()[moduleIndex];
    module.set(
      moduleState.getWheelVelocity(),
      new WrappedAngle(moduleState.getAzimuthPosition()),
      moduleState.getAzimuthVelocity()
    );
  }

  /**
   * Sets an individual module's velocity. The previous azimuth command is used
   * If you use this method. Don't call update()
   *
   * @param moduleIndex Index of the module
   * @param wheelVelocity Commanded wheel velocity of the module
   */
  public void setModuleVelocity(int moduleIndex, double wheelVelocity) {
    SwerveModule module = this.config.getModules()[moduleIndex];
    module.set(
      wheelVelocity,
      module.getAzimuthPosition()
    );
  }

  /**
   * Sets an individual module's azimuth. The previous wheel velocity command is used
   * If you use this method. Don't call update()
   *
   * @param moduleIndex Index of the module
   * @param azimuthPosition Commanded azimuth position of the module
   */
  public void setModuleAzimuth(int moduleIndex, double azimuthPosition) {
    SwerveModule module = this.config.getModules()[moduleIndex];
    module.set(
      module.getWheelVelocity(),
      new WrappedAngle(azimuthPosition));
  }

  /**
   * Gets the target velocity state.
   *
   * @return The target velocity state.
   */
  public VelocityState getTargetState() {
    return this.targetState;
  }

  /**
   * Gets the velocity state commanded after all constraints.
   *
   * @return The constrained velocity state.
   */
  public VelocityState getConstrainedCommandState() {
    return this.constrainedState;
  }

  /**
   * Sets a hold on the current module azimuth targets.
   *
   * @param hold True if there should be a hold, false otherwise.
   */
  public void holdAzimuths(boolean hold) {
    this.holdMode = hold;
  }

  /**
   * Gets if the chassis is in hold azimuth mode.
   *
   * @return True if the chassis is in hold azimuth mode, false otherwise.
   */
  public boolean inHoldAzimuthMode() {
    return this.holdMode;
  }

  /** Updates all periodic processes in the swerve chassis, such as setting module controls. */
  public void update() {
    // Update the forward kinematics and compute current pose
    this.forwardKinematics.update();

    // Constrain the target state
    VelocityState targetState = this.getTargetState();

    // Must be robot-centric
    VelocityState semiConstrainedState = this.makeRobotCentric(targetState);

    // Do not let any wheel exceed it's max speed
    semiConstrainedState = this.limitWheelSpeed(semiConstrainedState);

    // Set the constrained state
    this.constrainedState = semiConstrainedState;

    // Command the modules
    ModuleState moduleStates[] = this.inverseKinematics.calculate(constrainedState);
    for (int idx = 0; idx < moduleStates.length; idx++) {
      if (this.holdMode
          && this.constrainedState.getTranslationSpeed() == 0
          && this.constrainedState.getRotationVelocity() == 0) {
          setModuleVelocity(idx, 0.0);
      } else {
        setModuleState(idx, moduleStates[idx]);
      }
    }
  }

  /**
   * Gets the chassis odometry state.
   *
   * @return The odometry state
   */
  public OdomState getOdomState() {
    return this.forwardKinematics.getOdom();
  }

  /**
   * Sets the chassis odometry state. (for setting chassis initial conditions)
   *
   * @param state state to set the chassis to
   */
  public void setOdomState(OdomState state) {
    this.forwardKinematics.setOdom(state);
  }

  /**
   * Sets the chassis odometry state. (for setting chassis initial conditions)
   *
   * @param x The x position of the chassis, in feet.
   * @param y The y position of the chassis, in feet.
   */
  public void setOdomState(double x, double y) {
    OdomState state = new OdomState();
    state.setPosition(x, y);
    this.setOdomState(state);
  }

  /**
   * Sets the chassis odometry state. (for setting chassis initial conditions)
   *
   * @param x The x position of the chassis, in feet.
   * @param y The y position of the chassis, in feet.
   * @param theta The heading of the chasis, in degrees.
   */
  public void setOdomState(double x, double y, double theta) {
    OdomState state = new OdomState();
    state.setPosition(x, y);
    state.setTheta(theta);
    this.setOdomState(state);
  }

  /** Sets all motors to coast mode. */
  public void setCoast() {
    Stream.of(this.config.getModules()).forEach(m -> m.setCoast());
  }

  /** Sets all motors to brake mode. */
  public void setBrake() {
    Stream.of(this.config.getModules()).forEach(m -> m.setBrake());
  }

  /**
   * Gets the maximum translation speed if the drive is doing nothing else.
   *
   * @return The maximum translation speed, in feet per second.
   */
  public double getMaxTranslationSpeed() {
    return Stream.of(this.config.getModules())
        .map(m -> m.getMaxWheelSpeed())
        .min(Double::compare)
        .get();
  }

  /**
   * Gets the maximum rotation speed if the drive is doing nothing else.
   *
   * @return The maximum rotation speed, in feet per second.
   */
  public double getMaxRotationSpeed() {
    return Stream.of(this.config.getModules())
        .map(
            module -> {
              Vector2D centerOfRotation = this.getTargetState().getCenterOfRotationVector();
              double distanceToCenter =
                  module.getLocation().plus(centerOfRotation.times(-1)).getMagnitude();
              return (module.getMaxWheelSpeed() / (2. * distanceToCenter * Math.PI)) * 360.;
            })
        .min(Double::compare)
        .get();
  }

  /**
   * Converts a velocity state to robot-centric using the gyro if it is field centric, otherwise
   * returns it unmodified.
   *
   * @param state The state to make robot-centric.
   * @return A robot-centric state.
   */
  private VelocityState makeRobotCentric(VelocityState state) {
    if (!state.isFieldCentric()) {
      return state;
    }
    return state
        .changeTranslationDirection(
            state.getTranslationDirection() - this.config.getGyro().getYaw())
        .changeIsFieldCentric(false);
  }

  /**
   * Limits the translation and rotation velocities such that no wheel exceeds it's max speed.
   *
   * @param state The state to limit.
   * @return The limited state.
   */
  private VelocityState limitWheelSpeed(VelocityState state) {
    SwerveModule[] modules = this.config.getModules();

    // Get the wheel speeds from both translation and rotation.
    Vector2D translationVector = this.inverseKinematics.calculateModuleTranslationVector(state);
    Vector2D[] rotationVectors =
        Stream.of(modules)
            .map((m) -> this.inverseKinematics.calculateModuleRotationVectors(state, m))
            .toArray(Vector2D[]::new);

    // Determine the module with the highest ratio of desired speed to max speed.
    double speedFactor = 0.;
    for (int i = 0; i < modules.length; i++) {
      Vector2D fullVector = translationVector.plus(rotationVectors[i]);
      double individualSpeedFactor = fullVector.getMagnitude() / modules[i].getMaxWheelSpeed();
      if (individualSpeedFactor > speedFactor) {
        speedFactor = individualSpeedFactor;
      }
    }

    // If no wheels are exceeding their max speed, just return the original state.
    if (speedFactor <= 1) {
      return state;
    }

    // Reducing both translation and rotation by the speedFactor will result in
    // the fastest wheel being at exactly max speed.
    return new VelocityState(
        state.getTranslationDirection(),
        state.getTranslationSpeed() / speedFactor,
        state.getRotationVelocity() / speedFactor,
        state.isFieldCentric());
  }
}
