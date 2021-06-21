package frc.team88.swerve.gyro;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Pigeon implements SwerveGyro {

  // The navx object that this is based on.
  private PigeonIMU base;

  // The offset to add to the yaw
  private double offset = 0;

  /** Constructor. Pigeon doesn't really have a default assumes CAN ID 0. */
  public Pigeon() {
    this.base = new PigeonIMU(0);
  }

  /**
   * Constructor. Uses the given PigeonIMU object instead of constructing a new one.
   *
   * @param base The AHRS object to base this object on.
   */
  public Pigeon(PigeonIMU base) {
    this.base = base;
  }

  /**
   * Construct. Uses the given CAN ID.
   *
   * @param id The CAN ID
   */
  public Pigeon(Integer id) {
    this.base = new PigeonIMU(id);
  }

  /**
   * Constructor. Uses the given TalonSRX controller.
   *
   * @param controller The TalonSRX controller.
   */
  public Pigeon(TalonSRX controller) {
    this.base = new PigeonIMU(controller);
  }

  @Override
  public double getYaw() {
    final double[] ypr = new double[3];
    base.getYawPitchRoll(ypr);
    return Math.IEEEremainder(ypr[0], 360) + this.offset;
  }

  @Override
  public double getYawRate() {
    final double[] xyz = new double[3];
    base.getRawGyro(xyz);
    return xyz[2];
  }

  @Override
  public void calibrateYaw(double yaw) {
    this.offset = yaw - this.getYaw() + this.offset;
  }
}
