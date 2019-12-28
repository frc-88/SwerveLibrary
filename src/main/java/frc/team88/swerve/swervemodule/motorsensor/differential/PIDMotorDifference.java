package frc.team88.swerve.swervemodule.motorsensor.differential;

import java.util.Objects;

import frc.team88.swerve.swervemodule.motorsensor.PIDMotor;

/**
 * Controls and senses the difference between 2 motors in a differential
 * mechanism. Handles all operations that can be done without coordinating with
 * the sum "motor", such as sensor reading, and delegates the rest to the
 * differential mechanism.
 */
public class PIDMotorDifference implements PIDMotor {

    // The differential mechanism this derives from
    DifferentialMechanism differential;

    // The offset to add to position values.
    private double offset = 0;

    /**
     * Constructor.
     * 
     * @param differential
     *                         The differential mechanism this derives from
     */
    public PIDMotorDifference(DifferentialMechanism differential) {
        this.differential = Objects.requireNonNull(differential);
    }

    @Override
    public double getPosition() {
        return (this.differential.getPositiveMotor().getPosition() - this.differential.getNegativeMotor().getPosition())
                / 2. + this.offset;
    }

    @Override
    public double getVelocity() {
        return (this.differential.getPositiveMotor().getVelocity() - this.differential.getNegativeMotor().getVelocity())
                / 2.;
    }

    @Override
    public void calibratePosition(double position) {
        this.offset = position - this.getPosition() + this.offset;
    }

    @Override
    public void setVelocity(double velocity) {
        this.differential.setDifferenceVelocity(velocity);
    }
}
