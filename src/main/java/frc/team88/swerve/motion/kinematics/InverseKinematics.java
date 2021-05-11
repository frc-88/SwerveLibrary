package frc.team88.swerve.motion.kinematics;

import java.util.Objects;

import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.state.ModuleState;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;

/**
 * Handles the mathematical processing of inverse kinematics for a swerve drive.
 * All calculations are robot-centric.
 */
public class InverseKinematics {

    // The modules being controlled.
    private SwerveModule[] modules;

    /**
     * Constructor.
     * 
     * @param modules
     *                    The modules to be controlled. Minimumn 2.
     */
    public InverseKinematics(SwerveModule... modules) {
        if (modules.length < 2) {
            throw new IllegalArgumentException("Cannot do inverse kinematics with less than 2 modules");
        }
        this.modules = modules;
    }

    /**
     * Sets the target motion state.
     * 
     * @param target
     *                   The motion state to target. Robot-centric.
     * @return The calculated module states.
     */
    public ModuleState[] calculate(VelocityState target) {
        Objects.requireNonNull(target);
        if (target.isFieldCentric()) {
            throw new IllegalArgumentException("Cannot give field-centric velocity state to inverse kinematics");
        }
        
        // Get the translation vector, which is the same for all modules
        Vector2D translationVector = calculateModuleTranslationVector(target);

        ModuleState moduleStates[] = new ModuleState[this.modules.length];
        for (int idx = 0; idx < this.modules.length; ++idx) {
            Vector2D rotationVector = calculateModuleRotationVectors(target, modules[idx]);
            Vector2D combinedVector = translationVector.plus(rotationVector);
            if (translationVector.getMagnitude() == 0) {
                moduleStates[idx] = new ModuleState(target.getTranslationDirection(), 0);
            } else {
                moduleStates[idx] = new ModuleState(combinedVector.getAngle().asDouble(), combinedVector.getMagnitude());
            }
        }
        
        return moduleStates;
    }

    /**
     * Calculates the velocity vector for any module corresponding to the
     * translation component of motion.
     * 
     * @param state The target velocity state.
     * @return The velocity vector in feet per second.
     */
    protected Vector2D calculateModuleTranslationVector(VelocityState state) {
        // The translation vector for each module is the same as the overall
        // translation vector
        return Vector2D.createPolarCoordinates(state.getTranslationSpeed(), new WrappedAngle(state.getTranslationDirection()));
    }

    /**
     * Calculates the velocity vector for the given module corresponding to the
     * rotation component of motion.
     * 
     * @param state The target velocity state.
     * @param module The swerve module to calculate for.
     * @return The velocity vector in feet per second.
     */
    protected Vector2D calculateModuleRotationVectors(VelocityState state, SwerveModule module) {
        // Get the location of the module relative to the robot's origin
        Vector2D location = module.getLocation();
        // Calculate the location relative to the center of rotation.
        // (location - centerOfRotation)
        Vector2D positionFromRotationCenter = location.plus(state.getCenterOfRotationVector().times(-1));
        // The direction to move is perpendicular to the position vector
        Vector2D unscaled = positionFromRotationCenter.rotate(90);
        // For every full rotation of the robot, each wheel needs to drive
        // once around the circumference of the circle defined by the center
        // of rotation at it's center and the location of the module on the
        // circle. Therefore, the velocity for w rotations per second is w
        // times the circumference of the circle, and 1/360th of that for
        // w degrees per second.
        double circumference = positionFromRotationCenter.getMagnitude() * 2. * Math.PI;
        return unscaled.changeMagnitude(state.getRotationVelocity() * circumference / 360.);
    }
}