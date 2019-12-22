package frc.team88.swerve.swervemodule.state;

import java.util.Objects;

import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.util.Vector2D;

/**
 * Represents a target module state which specifies only wheel speed and the
 * azimuth angle.
 */
public class WheelSpeedAzimuthAngleModuleState implements ModuleState {

    // The target state, as a velocity vector
    private Vector2D velocity;

    /**
     * Constructor.
     * @param state The state to set as a velocity vector, where the direction
     * is the module angle and the magnitude is the wheel speed
     */
    public WheelSpeedAzimuthAngleModuleState(Vector2D velocity) {
        this.velocity = Objects.requireNonNull(velocity);
    }

    @Override
    public void applyToModule(SwerveModule module) {
        module.setWheelSpeed(velocity.getMagnitude());
        module.setAzimuthPosition(velocity.getAngle());
	}

}