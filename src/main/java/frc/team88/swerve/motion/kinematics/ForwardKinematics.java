package frc.team88.swerve.motion.kinematics;

import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.util.RobotControllerWrapper;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;
import org.apache.commons.math3.linear.RealMatrixFormat;

// Heavily influenced by these wpilib classes:
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;

/** Does the calculation to convert for sensed module state to robot position/velocity. */
public class ForwardKinematics {
  // The modules being controlled.
  private SwerveModule[] modules;

  private OdomState m_state;
  private RealMatrix forwardKinematics;
  private RealMatrix inverseKinematics;

  private RealMatrix moduleStatesMatrix;
  private RealMatrix poseRotationMatrix;
  private RealMatrix poseTranslationMatrix;
  private RealMatrix poseVector;
  private RealMatrix deltaPoseVector;

  // The last time the kinematics were calculated, in seconds.
  private double previousTime_s = RobotControllerWrapper.getInstance().getFPGATime() * 1E-6;

  private final double kTimeJumpThreshold = 10.0;

  /**
   * Constructor.
   *
   * @param modules The modules on this swerve drive.
   */
  public ForwardKinematics(SwerveModule... modules) {
    if (modules.length < 2) {
      throw new IllegalArgumentException("Cannot do forward kinematics with less than 2 modules");
    }
    this.modules = modules;
    m_state = new OdomState();

    inverseKinematics = new Array2DRowRealMatrix(modules.length * 2, 3);
    moduleStatesMatrix = new Array2DRowRealMatrix(modules.length * 2, 1);

    poseVector = new Array2DRowRealMatrix(3, 1);
    deltaPoseVector = new Array2DRowRealMatrix(3, 1);

    /*  Matrix format:
        [
            [cos(th), -sin(th), 0],
            [sin(th), cos(th), 0],
            [0, 0, 1]
        ]
    */
    poseRotationMatrix = new Array2DRowRealMatrix(3, 3);
    poseRotationMatrix.setEntry(0, 2, 0);
    poseRotationMatrix.setEntry(1, 2, 0);
    poseRotationMatrix.setEntry(2, 2, 1);
    poseRotationMatrix.setEntry(2, 0, 0);
    poseRotationMatrix.setEntry(2, 1, 0);

    /*  Matrix format:
        [
            [sin(dth)/dth, (cos(dth) - 1)/dth, 0],
            [(1 - cos(dth))/dth, sin(dth)/dth, 0],
            [0, 0, 1]
        ]
    */
    poseTranslationMatrix = new Array2DRowRealMatrix(3, 3);
    poseTranslationMatrix.setEntry(0, 2, 0);
    poseTranslationMatrix.setEntry(1, 2, 0);
    poseTranslationMatrix.setEntry(2, 2, 1);
    poseTranslationMatrix.setEntry(2, 0, 0);
    poseTranslationMatrix.setEntry(2, 1, 0);

    /*  Matrix format:
        [
            [1, 0, -module_n0_y],
            [0, 1, module_n0_x],
            [1, 0, -module_n1_y],
            [0, 1, module_n1_x],
            ...
        ]
    */
    for (int idx = 0; idx < modules.length; idx++) {
      Vector2D module_loc = modules[idx].getLocation();
      inverseKinematics.setEntry(idx * 2, 0, 1);
      inverseKinematics.setEntry(idx * 2, 1, 0);
      inverseKinematics.setEntry(idx * 2, 2, -module_loc.getY());
      inverseKinematics.setEntry(idx * 2 + 1, 0, 0);
      inverseKinematics.setEntry(idx * 2 + 1, 1, 1);
      inverseKinematics.setEntry(idx * 2 + 1, 2, module_loc.getX());
    }
    SingularValueDecomposition svd = new SingularValueDecomposition(inverseKinematics);
    DecompositionSolver solver = svd.getSolver();
    forwardKinematics = solver.getInverse();
    RealMatrixFormat TABLE_FORMAT = new RealMatrixFormat("", "", "", "\n", "", ", ");
    System.out.println("inverseKinematics:");
    System.out.println(TABLE_FORMAT.format(inverseKinematics));
    System.out.println("forwardKinematics:");
    System.out.println(TABLE_FORMAT.format(forwardKinematics));
  }

  /** Update the current robot pose. */
  public void update() {
    calculateChassisVector();
    estimatePoseExponential();
  }

  /**
   * Get the current robot pose.
   *
   * @return The current robot pose.
   */
  public OdomState getOdom() {
    return m_state;
  }

  /**
   * Set the current robot pose.
   *
   * @param state The current robot pose.
   */
  public void setOdom(OdomState state) {
    this.m_state = state;
  }

  /** Calculate the velocities of the chassis. */
  private void calculateChassisVector() {
    for (int idx = 0; idx < modules.length; idx++) {
      WrappedAngle azimuth = modules[idx].getAzimuthPosition();
      double wheel_speed = modules[idx].getWheelVelocity();

      double azimuthRad = Math.toRadians(azimuth.asDouble());

      double vx = wheel_speed * Math.cos(azimuthRad);
      double vy = wheel_speed * Math.sin(azimuthRad);
      moduleStatesMatrix.setEntry(idx * 2, 0, vx);
      moduleStatesMatrix.setEntry(idx * 2 + 1, 0, vy);
    }
    RealMatrix chassisVector = forwardKinematics.multiply(moduleStatesMatrix);
    m_state.setVelocity(chassisVector.getEntry(0, 0), chassisVector.getEntry(1, 0));
    m_state.setThetaVelocity(Math.toDegrees(chassisVector.getEntry(2, 0)));
  }

  /**
   * Calculate the position of the robot.
   *
   * <p>See https://file.tavsys.net/control/controls-engineering-in-frc.pdf Section 10.2 "Pose
   * exponential" for the theory and derivation Takes vx, vy, vt calculated in
   * calculateChassisVector and stored in state. Computes the next x, y, and t pose
   */
  private void estimatePoseExponential() {
    double currentTime_s = RobotControllerWrapper.getInstance().getFPGATime() * 1E-6;
    double dt = currentTime_s - previousTime_s;
    previousTime_s = currentTime_s;
    if (dt > kTimeJumpThreshold || dt <= 0.) {  // ignore cases where the clock jumps forward or backwards suddenly
      return;
    }

    double dx = m_state.getXVelocity() * dt;
    double dy = m_state.getYVelocity() * dt;
    double dtheta = Math.toRadians(m_state.getThetaVelocity()) * dt;
    
    double sin_dtheta = Math.sin(dtheta);
    double cos_dtheta = Math.cos(dtheta);

    double theta = Math.toRadians(m_state.getTheta()) + dtheta;
    double sin_theta = Math.sin(theta);
    double cos_theta = Math.cos(theta);

    double s;
    double c1;
    double c2;
    // Transformation from twist to pose can be indeterminant when angular velocity is zero.
    // Use taylor series approximation to mitigate this problem.
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c1 = 0.5 * dtheta;
      c2 = -0.5 * dtheta;
    } else {
      s = sin_dtheta / dtheta;
      c1 = (1 - cos_dtheta) / dtheta;
      c2 = (cos_dtheta - 1) / dtheta;
    }

    /*  Matrix format:
        [
            [cos(th), -sin(th), 0],
            [sin(th), cos(th), 0],
            [0, 0, 1]
        ]
    */
    poseRotationMatrix.setEntry(0, 0, cos_theta);
    poseRotationMatrix.setEntry(0, 1, -sin_theta);
    poseRotationMatrix.setEntry(1, 0, sin_theta);
    poseRotationMatrix.setEntry(1, 1, cos_theta);

    /*  Matrix format:
        [
            [sin(dth)/dth, (cos(dth) - 1)/dth, 0],
            [(1 - cos(dth))/dth, sin(dth)/dth, 0],
            [0, 0, 1]
        ]
    */
    poseTranslationMatrix.setEntry(0, 0, s);
    poseTranslationMatrix.setEntry(0, 1, c1);
    poseTranslationMatrix.setEntry(1, 0, c2);
    poseTranslationMatrix.setEntry(1, 1, s);

    poseVector.setEntry(0, 0, dx);
    poseVector.setEntry(1, 0, dy);
    poseVector.setEntry(2, 0, dtheta);

    deltaPoseVector = poseRotationMatrix.multiply(poseTranslationMatrix.multiply(poseVector));

    m_state.addToPosition(deltaPoseVector.getEntry(0, 0), deltaPoseVector.getEntry(1, 0));
    m_state.addToTheta(Math.toDegrees(deltaPoseVector.getEntry(2, 0)));
  }
}
