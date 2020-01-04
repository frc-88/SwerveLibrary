package frc.team88.swerve.swervemodule;

import java.util.Objects;

import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;

/**
 * Represents the state of a module, with azimuth position, azimuth velocity,
 * and wheel velocity. Immutable.
 */
public class ModuleState {

    // The rate of change of the module azimuth, in degrees per second.
    private final double azimuthVelocity;

    // The module azimuth, in degrees.
    private final WrappedAngle azimuthPosition;

    // The wheel speed, in feet per second.
    private final double wheelVelocity;

    /**
     * Constructor.
     * 
     * @param azimuthVelocity The rate of change of the module azimuth, in degrees
     *                        per second
     * @param azimuthPosition The module azimuth, in degrees
     * @param wheelVelocity   The wheel speed, in feet per second
     */
    public ModuleState(double azimuthVelocity, WrappedAngle azimuthPosition, double wheelVelocity) {
        this.azimuthVelocity = azimuthVelocity;
        this.azimuthPosition = Objects.requireNonNull(azimuthPosition);
        this.wheelVelocity = wheelVelocity;
    }

    /**
     * Constructor.
     * 
     * @param azimuthVelocity The rate of change of the module azimuth, in degrees
     *                        per second
     * @param velocityVector  A velocity vector where the angle represents azimuth
     *                        position and the magnitude represents wheel velocity
     */
    public ModuleState(double azimuthVelocity, Vector2D velocityVector) {
        this.azimuthVelocity = azimuthVelocity;
        Objects.requireNonNull(velocityVector);
        this.azimuthPosition = velocityVector.getAngle();
        this.wheelVelocity = velocityVector.getMagnitude();
    }

    /**
     * Create a new module state that is the same as this one, except with the given
     * azimuth velocity.
     * 
     * @param newVelocity The rate of change of the module azimuth, in degrees per
     *                    second
     * @return The new module state
     */
    public ModuleState changeAzimuthVelocity(double newVelocity) {
        return new ModuleState(newVelocity, this.getAzimuthPosition(), this.getWheelVelocity());
    }

    /**
     * Create a new module state that is the same as this one, except with the given
     * azimuth position.
     * 
     * @param newPosition The module azimuth, in degrees
     * @return The new module state
     */
    public ModuleState changeAzimuthPosition(WrappedAngle newPosition) {
        return new ModuleState(this.getAzimuthVelocity(), newPosition, this.getWheelVelocity());
    }

    /**
     * Create a new module state that is the same as this one, except with the given
     * wheel velocity.
     * 
     * @param newVelocity The wheel speed, in feet per second
     * @return The new module state
     */
    public ModuleState changeWheelVelocity(double newVelocity) {
        return new ModuleState(this.getAzimuthVelocity(), this.getAzimuthPosition(), newVelocity);
    }

    /**
     * Create a new module state that is the same as this one, except with the given
     * velocity vector.
     * 
     * @param newVelocityVector A velocity vector where the angle represents azimuth
     *                          position and the magnitude represents wheel velocity
     * @return The new module state
     */
    public ModuleState changeVelocityVector(Vector2D newVelocityVector) {
        return new ModuleState(this.getAzimuthVelocity(), newVelocityVector);
    }

    /**
     * Get the azimuth velocity for this module state.
     * 
     * @return The rate of change of the module azimuth, in degrees per second
     */
    public double getAzimuthVelocity() {
        return this.azimuthVelocity;
    }

    /**
     * Get the azimuth position for this module state.
     * 
     * @return The module azimuth, in degrees
     */
    public WrappedAngle getAzimuthPosition() {
        return this.azimuthPosition;
    }

    /**
     * Get the wheel velocity for this module state.
     * 
     * @return The wheel speed, in feet per second
     */
    public double getWheelVelocity() {
        return this.azimuthVelocity;
    }

    /**
     * Get the velocity vector representation of this module state.
     * 
     * @return A velocity vector where the angle represents azimuth position and the
     *         magnitude represents wheel velocity
     */
    public Vector2D asVelocityVector() {
        return Vector2D.createPolarCoordinates(this.getWheelVelocity(), this.getAzimuthPosition());
    }

}
