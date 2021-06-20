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
      SwerveModule module = this.config.getModules()[idx];
      if (this.holdMode
          && this.constrainedState.getTranslationSpeed() == 0
          && this.constrainedState.getRotationVelocity() == 0) {
        module.set(0, module.getAzimuthPosition());
      } else {
        module.set(
            moduleStates[idx].getWheelSpeed(),
            new WrappedAngle(moduleStates[idx].getAzimuthPosition()));
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

    // Get the wheel speeds without any limiting.
    ModuleState[] unlimitedModuleStates = this.inverseKinematics.calculate(state);

    // Determine the largest possible factor <=1 that when applied to all
    // wheel speeds will keep them below their max.
    double speedFactor = 1.;
    for (int i = 0; i < modules.length; i++) {
      double individualSpeedFactor =
          modules[i].getMaxWheelSpeed() / unlimitedModuleStates[i].getWheelSpeed();
      if (individualSpeedFactor < speedFactor) {
        speedFactor = individualSpeedFactor;
      }
    }

    // If no wheels are exceeding their max speed, just return the original state.
    if (speedFactor >= 0.99999) {
      return state;
    }

    // Apply the factor to all the wheels
    final double trueSpeedFactor = speedFactor; // Variables referenced in lambda must be final
    ModuleState[] limitedModuleStates =
        Stream.of(unlimitedModuleStates)
            .map(
                (s) -> new ModuleState(s.getAzimuthPosition(), s.getWheelSpeed() * trueSpeedFactor))
            .toArray(ModuleState[]::new);

    // Use forwards kinematics to determine the corresponding velocity state
    return this.forwardKinematics.calculateChassisVector(limitedModuleStates);
  }
}
