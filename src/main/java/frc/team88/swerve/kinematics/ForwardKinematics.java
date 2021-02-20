package frc.team88.swerve.kinematics;

import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;

import java.lang.Math;

public class ForwardKinematics
{
    // The modules being controlled.
    private SwerveModule[] modules;

    private OdomState state;
    private RealMatrix forwardKinematics; 
    private RealMatrix inverseKinematics;

    private RealMatrix moduleStatesMatrix;
    private RealMatrix poseRotationMatrix;
    private RealMatrix poseTranslationMatrix;
    private RealMatrix poseVector;
    private RealMatrix deltaPoseVector;


    public ForwardKinematics(SwerveModule... modules)
    {
        if (modules.length < 2) {
            throw new IllegalArgumentException("Cannot do forward kinematics with less than 2 modules");
        }
        this.modules = modules;
        state = new OdomState();

        inverseKinematics = new Array2DRowRealMatrix(modules.length, 3);
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
        for (int idx = 0; idx < modules.length; ++idx) {
            Vector2D module_loc = modules[idx].getLocation();
            inverseKinematics.setEntry(idx * 2, 0, 1);
            inverseKinematics.setEntry(idx * 2, 1, 1);
            inverseKinematics.setEntry(idx * 2, 2, -module_loc.getY());
            inverseKinematics.setEntry(idx * 2 + 1, 0, 1);
            inverseKinematics.setEntry(idx * 2 + 1, 1, 1);
            inverseKinematics.setEntry(idx * 2 + 1, 2, module_loc.getX());
        }
        SingularValueDecomposition svd = new SingularValueDecomposition(inverseKinematics);
        DecompositionSolver solver = svd.getSolver();
        forwardKinematics = solver.getInverse();
    }

    private void calculateChassisVector()
    {
        for (int idx = 0; idx < modules.length; ++idx) {
            WrappedAngle azimuth = modules[idx].getAzimuthPosition();
            double wheel_speed = modules[idx].getWheelSpeed();

            double vx = wheel_speed * Math.cos(azimuth.asDouble());
            double vy = wheel_speed * Math.sin(azimuth.asDouble());
            moduleStatesMatrix.setEntry(idx * 2, 0, vx);
            moduleStatesMatrix.setEntry(idx * 2 + 1, 0, vy);
        }
        RealMatrix chassisVector = forwardKinematics.multiply(moduleStatesMatrix);
        state.vx = chassisVector.getEntry(0, 0);
        state.vy = chassisVector.getEntry(0, 1);
        state.vt = chassisVector.getEntry(0, 2);
    }

    /**
     * See https://file.tavsys.net/control/controls-engineering-in-frc.pdf
     * Section 10.2 "Pose exponential" for the theory and derivation
     * Takes vx, vy, vt calculated in calculateChassisVector and stored in state.
     * Computes the next x, y, and t pose
     */
    private void estimatePoseExponential(double dt)
    {
        if (dt <= 0.0) {
            return;
        }
        double dx = state.vx * dt;
        double dy = state.vy * dt;
        double dtheta = state.vt * dt;

        double sin_theta = Math.sin(dtheta);
        double cos_theta = Math.cos(dtheta);

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
            s = sin_theta / dtheta;
            c1 = (1 - cos_theta) / dtheta;
            c2 = (cos_theta - 1) / dtheta;
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

        state.x += deltaPoseVector.getEntry(0, 0);
        state.y += deltaPoseVector.getEntry(1, 0);
        state.t += deltaPoseVector.getEntry(2, 0);
    }

    public void update(double dt)
    {
        calculateChassisVector();
        estimatePoseExponential(dt);
    }

    public OdomState getOdom() {
        return state;
    }

    public void setOdom(OdomState state) {
        this.state = state;
    }
}
