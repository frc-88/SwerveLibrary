package frc.team88.swerve.swervemodule.motorsensor.differential;

import java.util.Objects;

import frc.team88.swerve.swervemodule.motorsensor.PIDMotor;

/**
 * Takes a pair of PID motors and represents them in a differential mechanism,
 * creating PID "motors" that represent their sum and difference. Handles the
 * operations that require coorination between motors, such as setting the
 * velocity.
 */
public class DifferentialMechanism {

    // The motor that is subtracted from in the difference
    private PIDMotor positiveMotor;

    // The motor that is subtracted in the difference
    private PIDMotor negativeMotor;

    // The difference between the 2 motors
    private PIDMotorDifference differenceMotor;

    // The sum between the 2 motors
    private PIDMotorSum sumMotor;

    // The currently set difference velocity
    private double differenceVelocity = 0;

    // The currently set sum velocity
    private double sumVelocity = 0;

    /**
     * Constructor.
     * @param positiveMotor The motor that is subtracted from in the difference
     * @param negativeMotor The motor that is subtracted in the difference
     */
    public DifferentialMechanism(PIDMotor positiveMotor, 
            PIDMotor negativeMotor) {
        this.positiveMotor = Objects.requireNonNull(positiveMotor);
        this.negativeMotor = Objects.requireNonNull(negativeMotor);

        this.differenceMotor = new PIDMotorDifference(this);
        this.sumMotor = new PIDMotorSum(this);
    }

    /**
     * Gets the positive motor in this differential mechanism.
     * @return The motor that is subtracted from in the difference
     */
    public PIDMotor getPositiveMotor() {
        return positiveMotor;
    }

    /**
     * Gets the negative motor in this differential mechanism.
     * @return The motor that is subtracted in the difference
     */
    public PIDMotor getNegativeMotor() {
        return negativeMotor;
    }

    /**
     * Gets the PIDMotor representation of the difference in this differential
     * mechanism.
     * @return The difference "motor"
     */
    public PIDMotorDifference getDifferenceMotor() {
        return this.differenceMotor;
    }

    /**
     * Gets the PIDMotor representation of the sum in this differential
     * mechanism.
     * @return The sum "motor"
     */
    public PIDMotorSum getSumMotor() {
        return this.sumMotor;
    }

    /**
     * Sets the velocity of the difference.
     * @param velocity The difference velocity
     */
    public void setDifferenceVelocity(double velocity) {
        this.differenceVelocity = velocity;
        this.updateVelocityController();
    }

    
    /**
     * Sets the velocity of the sum.
     * @param velocity The sum velocity
     */
    public void setSumVelocity(double velocity) {
        this.sumVelocity = velocity;
        this.updateVelocityController();
    }

    /**
     * Updates the velocity control to use the most recently set velocity
     * values.
     */
    public void updateVelocityController() {
        this.getPositiveMotor().setVelocity(sumVelocity + differenceVelocity);
        this.getNegativeMotor().setVelocity(sumVelocity - differenceVelocity);
    }

}