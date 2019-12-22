package frc.team88.swerve.swervemodule.state;

import frc.team88.swerve.swervemodule.SwerveModule;

/**
 * Represents the target state of a swerve module from a high level
 * perspective. Knows how to set a SwerveModule to target it.
 */
public interface ModuleState {

    /**
     * Sets the give swerve module to target this state.
     * @param module The module to set
     */
    public void applyToModule(SwerveModule module);

}