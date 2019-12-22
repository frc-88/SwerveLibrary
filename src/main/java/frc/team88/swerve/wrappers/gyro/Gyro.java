package frc.team88.swerve.wrappers.gyro;

/**
 * Represents a gyroscope. Values returned follow our rules for units and the
 * field coordinate frame. Specifically, units are in degrees and degrees per
 * second, and are defined as follows:
 * 
 * <p>
 * "An angle of 0 degrees is defined as pointing in the direction of the
 * positive y-axis, increasing positively in the counter-clockwise direction and
 * negatively in the clockwise direction. The positive x-axis is at -90
 * degrees."
 * </p>
 * 
 * <p>
 * Technically classes which implement this interface have no way to enforce
 * where the 0 angle points, but it is expected that users will follow this
 * rule.
 * </p>
 */
public interface Gyro {

    /**
     * Gets the current yaw.
     * 
     * @return the current yaw, in degrees where positive angles are
     *         counterclockwise and angle 0 is at the positive Y axis
     */
    public double getYaw();

    /**
     * Gets the current yaw rate of change.
     * 
     * @return the current yaw rate, in degrees per second where positive angles are
     *         counterclockwise
     */
    public double getYawRate();

    /**
     * Offsets the yaw value such that the current yaw reading will be the given
     * value.
     * 
     * @param yaw
     *                The yaw value to set
     */
    public void calibrateYaw(double yaw);
}