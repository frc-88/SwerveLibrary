package frc.team88.swerve.swervemodule.motorsensor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.team88.swerve.util.constants.PIDPreferenceConstants;

/**
 * PIDMotor implementation for the NEO. Uses the built-in encoder and PID on
 * the Spark Max.
 */
public class PIDNeo extends CANSparkMax implements PIDMotor {

    // All of the PID gains
    private PIDPreferenceConstants pidConstants;

    // The offset to add to position values, in rotations.
    private double offset = 0;

    /**
     * Constructor. Sets up the default configuration for a SparkMax.
     * @param canID The canID for the SparkMax
     * @param pidConstants All of the PID constants for velocity control. Note:
     * only uses kP, kI, kD, and kF
     */
    public PIDNeo(int canID, PIDPreferenceConstants pidConstants) {
        super(canID, MotorType.kBrushless);
        
        this.pidConstants = pidConstants;
        pidConstants.getKP().assignChangeHandler(this::setKP);
        pidConstants.getKI().assignChangeHandler(this::setKI);
        pidConstants.getKD().assignChangeHandler(this::setKD);
        pidConstants.getKF().assignChangeHandler(this::setKF);
        this.setKP(pidConstants.getKP().getValue());
        this.setKI(pidConstants.getKP().getValue());
        this.setKD(pidConstants.getKP().getValue());
        this.setKF(pidConstants.getKP().getValue());

        this.restoreFactoryDefaults();
        this.setIdleMode(IdleMode.kBrake);
    }

    /**
     * {@inheritDoc}
     * @return The motor shaft position in rotations
     */
    @Override
    public double getPosition() {
        return this.getEncoder().getPosition() + offset;
    }

    /**
     * {@inheritDoc}
     * @return The motor shaft position in rotations per second
     */
    @Override
    public double getVelocity() {
        return this.getEncoder().getVelocity() / 60.;
    }

    /**
     * {@inheritDoc}
     * @return The position to set, in rotations
     */
    @Override
    public void calibratePosition(double position) {
        this.offset = position - this.getPosition();
    }

    /**
     * {@inheritDoc}
     * @return The velocity to set, in rotations per second
     */
    @Override
    public void setVelocity(double velocity) {
        this.getPIDController().setReference(velocity * 60, 
                ControlType.kVelocity);
    }

    /**
     * Set the kP constant for the Spark Max velocity control.
     * @param kP The proportional gain
     */
    private void setKP(double kP) {
        this.getPIDController().setP(kP);
    }

    /**
     * Set the kI constant for the Spark Max velocity control.
     * @param kI The integral gain
     */
    private void setKI(double kI) {
        this.getPIDController().setI(kI);
    }

    /**
     * Set the kD constant for the Spark Max velocity control.
     * @param kD The differential gain
     */
    private void setKD(double kD) {
        this.getPIDController().setD(kD);
    }

    /**
     * Set the kF constant for the Spark Max velocity control.
     * @param kF The feedforward gain
     */
    private void setKF(double kF) {
        this.getPIDController().setFF(kF);
    }

}