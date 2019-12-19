package frc.team88.swerve.util.wpilibwrappers;

import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Wrapper around the WPILib RobotController class. Follows the singleton
 * pattern for testability.
 */
public class RobotControllerWrapper {

    // The singleton instance
    private RobotControllerWrapper instance;

    /**
     * Private constructor. Does nothing.
     */
    private RobotControllerWrapper() {
        // Do nothing
    }

    /**
     * Get the singleton instance
     * @return The singleton instance
     */
    public RobotControllerWrapper getInstance() {
        if (Objects.isNull(instance)) {
            instance = new RobotControllerWrapper();
        }
        return instance;
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