package frc.team88.swerve.module.sensor;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

/** Represents a cancoder. */
public class SwerveCANcoder implements PositionSensor {

  // The cancoder being wrapped
  private CANCoder cancoder;

  /**
   * Constructor.
   *
   * @param canID The CAN ID of the CANCoder.
   */
  public SwerveCANcoder(int canID) {
    this.cancoder = new CANCoder(canID);

    this.cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 5);
  }

  @Override
  public double getPosition() {
    return this.cancoder.getAbsolutePosition() / 360.;
  }
}
