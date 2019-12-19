package frc.team88.swerve.util.constants;

import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

/**
 * This class holds all of the constants for the Swerve Library.
 */
public class Constants {

    /** 
     * Updates all UpdatableConstants.
     */
    public static void update() {
        for (UpdatableConstant constant : updatableConstants) {
            constant.update();
        }
    }

    // The updatable constants registered.
    private static List<UpdatableConstant> updatableConstants;

    /**
     * Registers an updatable constant to be updated.
     * @param constant The constant to be registered
     */
    protected static void addUpdatableConstant(UpdatableConstant constant) {
        if (Objects.isNull(updatableConstants)) {
            updatableConstants = new LinkedList<>();
        }
        updatableConstants.add(constant);
    }
}
