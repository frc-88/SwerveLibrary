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

    // The port of the XBox gamepad.
    public static final int GAMEPAD_PORT = 0;

    // The max translation and rotation speeds to be commanded to the drivetrain.
    public static final double MAX_SPEED = 14.7; // feet per second
    public static final double MAX_ROTATION = 90.; // degrees per second

    
}
