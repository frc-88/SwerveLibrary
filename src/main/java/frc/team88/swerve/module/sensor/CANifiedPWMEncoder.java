package frc.team88.swerve.module.sensor;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.CANifierStatusFrame;

/** Represents a PWM-based encoder that is connected to a CANifier. */
public class CANifiedPWMEncoder implements PositionSensor {

  // The CANifier that the encoder is plugged into
  private CANifier canifier;

  // The channel that the encoder is plugged into
  private PWMChannel channel;

  /**
   * Constructor.
   *
   * @param canifier The CANifier that the encoder is plugged into
   * @param channel The channel that the encoder is plugged into
   */
  public CANifiedPWMEncoder(CANifier canifier, PWMChannel channel) {
    this.canifier = canifier;
    this.channel = channel;
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 5);
  }

  @Override
  public double getPosition() {
    double[] dutyAndPeriod = new double[2];
    this.canifier.getPWMInput(channel, dutyAndPeriod);
    if (dutyAndPeriod[1] == 0.) {
      // Sensor is unplugged
      return 0;
    }
    return dutyAndPeriod[0] / dutyAndPeriod[1];
  }
}
