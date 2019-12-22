package frc.team88.swerve.util;

import frc.team88.swerve.state.SwerveModuleState;
import frc.team88.swerve.state.VelocitySwerveState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.MathUtils;

public class SwerveUtils {

<<<<<<< HEAD
  /**
   * Calculate module wheel and azimuth velocities based on desired robot frame
   * velocity. Adapted from Ether's Derivation of Inverse Kinematics for Swerve.
   * 
   * @param chasses  The dimensions of the chassis as measured between wheel
   *                 centers, in feet
   * @param velocity The desired translational velocity, in feet per second
   * @param omega    The desired rotational velocity, in degrees CCW
   * @return
   */
  public static SwerveModuleState calculateWheels(Vector2D chassis, Vector2D velocity, double omega) {
    // Convert omega to radians for use in atan
    omega = omega * Math.PI / 180;

    // Calculate the half-chassis measures
    Vector2D halfChassis = Vector2D.createCartesianCoordinates(chassis.getX() / 2, chassis.getY() / 2);

    // Calculate intermediate values
    double a = velocity.getX() - omega * halfChassis.getY();
    double b = velocity.getX() + omega * halfChassis.getY();
    double c = velocity.getY() - omega * halfChassis.getX();
    double d = velocity.getY() + omega * halfChassis.getX();

    // Calculate individual module values
    double frs = Math.sqrt(b * b + c * c);
    double fra = Math.atan2(b, c) * 180 / Math.PI;
    double fls = Math.sqrt(b * b + d * d);
    double fla = Math.atan2(b, d) * 180 / Math.PI;
    double bls = Math.sqrt(a * a + d * d);
    double bla = Math.atan2(a, d) * 180 / Math.PI;
    double brs = Math.sqrt(a * a + c * c);
    double bra = Math.atan2(a, c) * 180 / Math.PI;

    return new SwerveModuleState(frs, fra, fls, fla, bls, bla, brs, bra);
  }

  public static VelocitySwerveState limitAccelerations(VelocitySwerveState current, VelocitySwerveState desired,
      double maxLinearAccel, double maxTranslationAngularAccel, double maxHeadingAngularAccel, long dt) {

    double dt_seconds = dt / 1_000_000.;

    double limitedLinearSpeed = MathUtils.limitChange(
        current.getTranslationVelocity().getMagnitude(),
        desired.getTranslationVelocity().getMagnitude(), maxLinearAccel / dt_seconds);
    double limitedTranslationAngularSpeed = MathUtils.limitChange(
        current.getTranslationVelocity().getAngle(),
        desired.getTranslationVelocity().getAngle(), maxTranslationAngularAccel / dt_seconds);
    double limitedHeadingAngularSpeed = MathUtils.limitChange(
        current.getRotationalVelocity(),
        desired.getRotationalVelocity(), maxHeadingAngularAccel / dt_seconds);

    return new VelocitySwerveState(
        Vector2D.createPolarCoordinates(limitedLinearSpeed, limitedTranslationAngularSpeed),
        limitedHeadingAngularSpeed);

  }
=======
    public static ModuleState
>>>>>>> d7e5f28... Remove ModuleState

}