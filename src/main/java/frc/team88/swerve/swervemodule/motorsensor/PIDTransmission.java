package frc.team88.swerve.swervemodule.motorsensor;

/**
 * Composed of a single PIDMotor, and applies a gear ratio to it.
 */
public class PIDTransmission extends SensorTransmission implements PIDMotor {

    // The motor driving the transmission
    private PIDMotor inputMotor;

    public PIDTransmission(PIDMotor inputMotor, double gearRatio) {
        super(inputMotor, gearRatio);
        this.inputMotor = inputMotor;
    }

    @Override
    public void setVelocity(double velocity) {
        inputMotor.setVelocity(applyReverseGearRatio(velocity));
    }

}
