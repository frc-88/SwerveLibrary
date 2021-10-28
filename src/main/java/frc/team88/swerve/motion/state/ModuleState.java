package frc.team88.swerve.motion.state;

/** Encapsulates the running state of a single swerve module. */
public class ModuleState {

  private final double azimuthPosition;
  private final double wheelVelocity;
  private final double azimuthVelocity;

  /**
   * Constructs a module state.
   *
   * @param azimuthPosition The angle of the module, in degrees.
   * @param azimuthVelocity The velocity of the module, in degrees per second.
   * @param wheelVelocity The speed of the wheel, in feet per second.
   */
  public ModuleState(double azimuthPosition, double wheelVelocity, double azimuthVelocity) {
    this.azimuthPosition = azimuthPosition;
    this.wheelVelocity = wheelVelocity;
    this.azimuthVelocity = azimuthVelocity;
  }

  /**
   * Constructs a module state. Azimuth velocity is set to zero
   *
   * @param azimuthPosition The angle of the module, in degrees.
   * @param wheelVelocity The speed of the wheel, in feet per second.
   */
  public ModuleState(double azimuthPosition, double wheelVelocity) {
    this.azimuthPosition = azimuthPosition;
    this.wheelVelocity = wheelVelocity;
    this.azimuthVelocity = 0.0;
  }

  /**
   * Gets the azimuth position.
   *
   * @return The angle of the module, in degrees.
   */
  public double getAzimuthPosition() {
    return this.azimuthPosition;
  }

  /**
   * Gets the azimuth velocity.
   *
   * @return The angular velocity of the module, in degrees per second.
   */
  public double getAzimuthVelocity() {
    return this.azimuthVelocity;
  }

  /**
   * Gets the wheel speed.
   *
   * @return The speed of the wheel, in feet per second.
   */
  public double getWheelVelocity() {
    return this.wheelVelocity;
  }
}
