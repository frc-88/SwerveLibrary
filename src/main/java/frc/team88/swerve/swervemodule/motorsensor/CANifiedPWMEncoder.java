package frc.team88.swerve.swervemodule.motorsensor;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifier.PWMChannel;

import frc.team88.swerve.util.constants.DoublePreferenceConstant;

/**
 * Represents a PWM-based encoder that is connected to a CANifier. Because there
 * isn't a great way to get the velocity here, a difference source is required.
 */
public class CANifiedPWMEncoder implements PositionVelocitySensor {

    // The CANifier that the encoder is plugged into
    private CANifier canifier;

    // The channel that the encoder is plugged into
    private PWMChannel channel;

    // Gets the current velocity of whatever this is measuring, in rotations
    private DoubleSupplier velocitySupplier;

    // The offset to add to position values, in rotations.
    private DoublePreferenceConstant offset;

    /**
     * Constructor.
     * 
     * @param canifier
     *                             The CANifier that the encoder is plugged into
     * @param channel
     *                             The channel that the encoder is plugged into
     * @param velocitySupplier
     *                             Gets the current velocity of whatever this is
     *                             measuring, in rotations
     * @param offset
     *                             The offset to add to position values, in
     *                             rotations
     */
    public CANifiedPWMEncoder(CANifier canifier, PWMChannel channel, DoubleSupplier velocitySupplier,
            DoublePreferenceConstant offset) {
        this.canifier = canifier;
        this.channel = channel;
        this.velocitySupplier = velocitySupplier;
        this.offset = offset;
        this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 5);
        this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 5);
        this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 5);
        this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 5);
    }

    @Override
    public double getPosition() {
        double[] dutyAndPeriod = new double[2];
        this.canifier.getPWMInput(channel, dutyAndPeriod);
        return dutyAndPeriod[0] / dutyAndPeriod[1] + this.offset.getValue();
    }

    @Override
    public double getVelocity() {
        return this.velocitySupplier.getAsDouble();
    }

    @Override
    public void calibratePosition(double position) {
        this.offset.setValue(position - this.getPosition() + this.offset.getValue());
    }

}
