package frc.team88.swerve;

import edu.wpi.first.wpilibj.Preferences;

/**
 * This class contains many of the constants for the robot. Mainly, those values
 * which should go here are either ones which are relevant in a global scope or
 * ones that will be registered as a WPILIB preference.
 */
public class Constants {

    // PID constants for the MK2 wheel speed controller running on the Spark Max
    // Configured through Preferences
    private static String name_mk2WheelKP = "MK2 Wheel kP";
    public static double mk2WheelKP = 0;
    private static String name_mk2WheelKI = "MK2 Wheel kI";
    public static double mk2WheelKI = 0.0001;
    private static String name_mk2WheelKD = "MK2 Wheel kD";
    public static double mk2WheelKD = 0;
    private static String name_mk2WheelKF = "MK2 Wheel kF";
    public static double mk2WheelKF = .08;
    private static String name_mk2WheelIZone = "MK2 Wheel IZone";
    public static double mk2WheelIZone = 2;
    private static String name_mk2WheelIMax = "MK2 Wheel IMax";
    public static double mk2WheelIMax = .2;
    private static String name_mk2WheelRamp = "MK2 Wheel Ramp";
    public static double mk2WheelRamp = .05;

    // PID constants for the MK2 azimuth controller using the Spark Max's "Smart
    // Motion"
    // trapezoidal motion profiling
    // Configured through Preferences
    private static String name_mk2AzimuthKP = "MK2 Azimuth kP";
    public static double mk2AzimuthKP = 0.001;
    private static String name_mk2AzimuthKI = "MK2 Azimuth kI";
    public static double mk2AzimuthKI = 0.00000;
    private static String name_mk2AzimuthKD = "MK2 Azimuth kD";
    public static double mk2AzimuthKD = 0;
    private static String name_mk2AzimuthRamp = "MK2 Azimuth Ramp";
    public static double mk2AzimuthRamp = 0;

    // Acceleration limits
    private static String name_linearAccelLimit = "Linear Accel";
    public static double linearAccelLimit = 12;
    private static String name_translationAngularAccelLimit = "Translate Angular Accel";
    public static double translationAngularAccelLimit = 180;
    private static String name_headingAngularAccelLimit = "Heading Angular Accel";
    public static double headingAngularAccelLimit = 180;

    // Drivebase parameters
    /// wheel center-to-center along x-axis in ft
    public static double drivebaseLength = 2.0625;
    /// wheel center-to-center along y-axis in ft
    public static double drivebaseWidth = 1.8125;

    // Initialize all of the preferences
    public static void init() {
        initializeDoublePreference(name_mk2WheelKP, mk2WheelKP);
        initializeDoublePreference(name_mk2WheelKI, mk2WheelKI);
        initializeDoublePreference(name_mk2WheelKD, mk2WheelKD);
        initializeDoublePreference(name_mk2WheelKF, mk2WheelKF);
        initializeDoublePreference(name_mk2WheelIZone, mk2WheelIZone);
        initializeDoublePreference(name_mk2WheelIMax, mk2WheelIMax);
        initializeDoublePreference(name_mk2WheelRamp, mk2WheelRamp);

        initializeDoublePreference(name_mk2AzimuthKP, mk2AzimuthKP);
        initializeDoublePreference(name_mk2AzimuthKI, mk2AzimuthKI);
        initializeDoublePreference(name_mk2AzimuthKD, mk2AzimuthKD);
        initializeDoublePreference(name_mk2AzimuthRamp, mk2AzimuthRamp);

        initializeDoublePreference(name_linearAccelLimit, linearAccelLimit);
        initializeDoublePreference(name_translationAngularAccelLimit, translationAngularAccelLimit);
        initializeDoublePreference(name_headingAngularAccelLimit, headingAngularAccelLimit);
    }

    public static void updatePreferences() {
        mk2WheelKP = updateDoublePreference(name_mk2WheelKP, mk2WheelKP);
        mk2WheelKI = updateDoublePreference(name_mk2WheelKI, mk2WheelKI);
        mk2WheelKD = updateDoublePreference(name_mk2WheelKD, mk2WheelKD);
        mk2WheelKF = updateDoublePreference(name_mk2WheelKF, mk2WheelKF);
        mk2WheelIZone = updateDoublePreference(name_mk2WheelIZone, mk2WheelIZone);
        mk2WheelIMax = updateDoublePreference(name_mk2WheelIMax, mk2WheelIMax);
        mk2WheelRamp = updateDoublePreference(name_mk2WheelRamp, mk2WheelRamp);

        mk2AzimuthKP = updateDoublePreference(name_mk2AzimuthKP, mk2AzimuthKP);
        mk2AzimuthKI = updateDoublePreference(name_mk2AzimuthKI, mk2AzimuthKI);
        mk2AzimuthKD = updateDoublePreference(name_mk2AzimuthKD, mk2AzimuthKD);
        mk2AzimuthRamp = updateDoublePreference(name_mk2AzimuthRamp, mk2AzimuthRamp);

        linearAccelLimit = updateDoublePreference(name_linearAccelLimit, linearAccelLimit);
        translationAngularAccelLimit = updateDoublePreference(name_translationAngularAccelLimit,
                translationAngularAccelLimit);
        headingAngularAccelLimit = updateDoublePreference(name_headingAngularAccelLimit, headingAngularAccelLimit);
    }

    private static void initializeDoublePreference(String name, double defaultValue) {
        Preferences prefs = Preferences.getInstance();
        if (!prefs.containsKey(name)) {
            prefs.putDouble(name, defaultValue);
        }
    }

    private static double updateDoublePreference(String name, double currentValue) {
        return Preferences.getInstance().getDouble(name, currentValue);
    }

}