package frc.team88.swerve.swervemodule.motorsensor;

/**
 * Motor with associated sensor that can be set to a velocity setpoint. Does not
 * provide PID configuration options, since those will vary depending on whether
 * the loop is being run on the RIO or the motor controller.
 */
public interface PIDMotor extends PositionVelocitySensor {

    /**
     * Set the velocity to PID to the given value. Only gauranteed to work when
     * enabled.
     * 
     * @param velocity
     *                     The velocity to set
     */
    public void setVelocity(double velocity);

}
