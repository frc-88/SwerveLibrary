package frc.team88.swerve.swervemodule.motorsensor;

import java.util.Objects;

/**
 * Composed of a single PIDMotor, and applies a gear ratio to it.
 */
public class PIDTransmission implements PIDMotor {

    // The motor driving the transmission
    private PIDMotor inputMotor;

    // The gear ratio; outputSpeed = baseMotorSpeed / gearRatio
    private double gearRatio;

    public PIDTransmission(PIDMotor inputMotor, double gearRatio) {
        this.inputMotor = Objects.requireNonNull(inputMotor);
        if (gearRatio <= 0 ) {
            throw new IllegalArgumentException("Gear ratio must be positive.");
        }
        this.gearRatio = gearRatio;
    }

    @Override
    public double getPosition() {
        return applyForwardGearRatio(inputMotor.getPosition());
    }

    @Override
    public double getVelocity() {
        return applyForwardGearRatio(inputMotor.getVelocity());
    }

    @Override
    public void calibratePosition(double position) {
        inputMotor.calibratePosition(applyReverseGearRatio(position));
    }

    @Override
    public void setVelocity(double velocity) {
        inputMotor.setVelocity(applyReverseGearRatio(velocity));
    }

    /**
     * Applies the gear ratio to get from an input value to an output value.
     * @param value The input value
     * @return The output value
     */
    private double applyForwardGearRatio(double value) {
        return value / gearRatio;
    }

    /**
     * Applies the gear ratio to get from an output value to an input value.
     * @param value The output value
     * @return The input value
     */
    private double applyReverseGearRatio(double value) {
        return value * gearRatio;
    }

}