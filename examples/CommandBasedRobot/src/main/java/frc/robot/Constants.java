// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    // The port of the XBox gamepad.
    public static final int GAMEPAD_PORT = 0;

    // The deadband to use for XBox gamepad joysticks.
    public static final double JOYSTICK_DEADBAND = 0.1;
  }

  public static final class DriveConstants {
    // The path to the swerve config file in the deploy directory.
    public static final String SWERVE_CONFIG = "swerve.toml";

    // The max translation and rotation speeds to be commanded to the drivetrain.
    public static final double MAX_SPEED = 14.7; // feet per second
    public static final double TURTLE_SPEED = 5.0; // feet per second
    public static final double MAX_ROTATION = 90.; // degrees per second

    // The threshold for left stick magnitude to change translation direction when using a gas pedal.
    public static final double CHANGE_DIRECTION_THRESHOLD = 0.25;

    // The speed thresholds for holding modules in their current direction.
    public static final double HOLD_DIRECTION_TRANSLATION_THRESHOLD = 0.1;
    public static final double HOLD_DIRECTION_ROTATION_THRESHOLD = 1;
  }
}
