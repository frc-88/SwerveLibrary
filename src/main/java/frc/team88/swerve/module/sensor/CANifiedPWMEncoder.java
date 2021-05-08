package frc.team88.swerve.module.sensor;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifier.PWMChannel;

/**
 * Represents a PWM-based encoder that is connected to a CANifier. Because there
 * isn't a great way to get the velocity here, a difference source is required.
 */
public class CANifiedPWMEncoder implements PositionSensor {

    // The CANifier that the encoder is plugged into
    private CANifier canifier;

    // The channel that the encoder is plugged into
    private PWMChannel channel;

    /**
     * Constructor.
     * 
     * @param canifier
     *                             The CANifier that the encoder is plugged into
     * @param channel
     *                             The channel that the encoder is plugged into
     * @param offset
     *                             The offset to add to position values, in
     *                             rotations
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
        return dutyAndPeriod[0] / dutyAndPeriod[1];
    }
}
