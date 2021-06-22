package frc.team88.swerve.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public class NavX implements SwerveGyro {

  // The navx object that this is based on.
  private AHRS base;

  // The offset to add to the yaw
  private double offset = 0;

  /** Constructor. Uses the default port of SPI on the MXP. */
  public NavX() {
    this.base = new AHRS();
  }

  /**
   * Constructor. Uses the given AHRS object instead of constructing a new one.
   *
   * @param base The AHRS object to base this object on.
   */
  public NavX(AHRS base) {
    this.base = base;
  }

  /**
   * Construct. Uses the given SPI port.
   *
   * @param port The SPI port
   */
  public NavX(SPI.Port port) {
    this.base = new AHRS(port);
  }

  /**
   * Constructor. Uses the given I2C port.
   *
   * @param port The I2C port
   */
  public NavX(I2C.Port port) {
    this.base = new AHRS(port);
  }

  /**
   * Constructor. Uses the given serial port.
   *
   * @param port The serial port
   */
  public NavX(SerialPort.Port port) {
    this.base = new AHRS(port);
  }

  @Override
  public double getYaw() {
    return -this.base.getYaw() + this.offset;
  }

  @Override
  public double getYawRate() {
    return -this.base.getRate();
  }

  @Override
  public void calibrateYaw(double yaw) {
    this.offset = yaw - this.getYaw() + this.offset;
  }
}
