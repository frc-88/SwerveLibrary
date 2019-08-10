package frc.team88.swerve.util;

public class MathUtils {

    public static double getReferenceAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        angle -= 180;
        return angle;
    }

    public static double limitChange(double current, double desired, double maxChange) {
        if (desired > current) {
            return Math.min(desired, current + maxChange);
        } else {
            return Math.max(desired, current - maxChange);
        }
    }
}