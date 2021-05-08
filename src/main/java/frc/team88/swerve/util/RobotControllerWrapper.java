package frc.team88.swerve.util;

import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Wrapper around the WPILib RobotController class. Follows the singleton
 * pattern for testability.
 */
public class RobotControllerWrapper {

    // The singleton instance
    private static RobotControllerWrapper instance;

    /**
     * Private constructor. Does nothing.
     */
    public RobotControllerWrapper() {
        // Do nothing
    }

    /**
     * Get the singleton instance
     * 
     * @return The singleton instance
     */
    public static RobotControllerWrapper getInstance() {
        if (Objects.isNull(instance)) {
            instance = new RobotControllerWrapper();
        }
        return instance;
    }

    /**
     * Sets the singleton instance. Used only for testing.
     * 
     * @param instance The new singleton instance to set.
     */
    public static void setInstance(RobotControllerWrapper instance) {
        RobotControllerWrapper.instance = instance;
    }

    /**
     * Read the microsecond timer from the FPGA.
     *
     * @return The current time in microseconds according to the FPGA.
     */
    public long getFPGATime() {
        return RobotController.getFPGATime();
    }

}
