package frc.team88.swerve.motion.state;

/**
 * Encapsulates the running state of a single swerve module.
 */
public class ModuleState {
    
    private final double azimuthPosition;
    private final double wheelSpeed;

    /**
     * Constructs a module state.
     * 
     * @param azimuthPosition The angle of the module, in degrees.
     * @param wheelSpeed The speed of the wheel, in feet per second.
     */
    public ModuleState(double azimuthPosition, double wheelSpeed) {
        this.azimuthPosition = azimuthPosition;
        this.wheelSpeed = wheelSpeed;
    }

    /**
     * Gets the azimuth position.
     * 
     * @return The angle of the module, in degrees.
     */
    public double getAzimuthPosition() {
        return this.azimuthPosition;
    }

    /**
     * Gets the wheel speed.
     * 
     * @return The speed of the wheel, in feet per second.
     */
    public double getWheelSpeed() {
        return this.wheelSpeed;
    }
}
