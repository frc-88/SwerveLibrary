package frc.team88.swerve;

import edu.wpi.first.wpilibj.RobotController;
import frc.team88.swerve.Constants;
import frc.team88.swerve.state.AbsoluteHeadingSwerveState;
import frc.team88.swerve.state.FullSwerveState;
import frc.team88.swerve.state.SwerveState;
import frc.team88.swerve.state.SwerveStateVisitor;
import frc.team88.swerve.state.VelocitySwerveState;
import frc.team88.swerve.SwerveTelemetry;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.SwerveUtils;

public abstract class SwerveChassis {

    // Robot centric
    private SwerveState desiredState;

    private MotionController motionController = new MotionController();

    private long lastUpdateTime;

    protected SwerveChassis() {
        desiredState = VelocitySwerveState.ZERO_STATE;
        lastUpdateTime = RobotController.getFPGATime();
    }

    public void setRobotCentricVelocityTargets(double xVelocity, 
            double yVelocity, double rotationalVelocity) {

        desiredState = new VelocitySwerveState(
            Vector2D.createCartesianCoordinates(xVelocity, yVelocity), rotationalVelocity)
            .rotateFrame(-SwerveTelemetry.getInstance().getIMUHeading());

    }

    public void setFieldCentricVelocityTargets(double xVelocity, 
            double yVelocity, double rotationalVelocity) {

        desiredState = new VelocitySwerveState(
            Vector2D.createCartesianCoordinates(xVelocity, yVelocity), rotationalVelocity);

    }

    public void update() {
        this.desiredState.accept(this.motionController);
        lastUpdateTime = RobotController.getFPGATime();
    }

    private class MotionController implements SwerveStateVisitor<Void> {

        @Override
        public Void visitVelocitySwerveState(VelocitySwerveState desired) {

            VelocitySwerveState commanded = desired;
            FullSwerveState current = SwerveTelemetry.getInstance().getRobotCentricState();
            
            SwerveUtils.limitAccelerations(current, desired, Constants.linearAccelLimit, 
                Constants.translationAngularAccelLimit, Constants.headingAngularAccelLimit, 
                RobotController.getFPGATime() - lastUpdateTime);

            // TODO Convert to module commands

            return null;
        }

        @Override
        public Void visitAbsoluteHeadingSwerveState(AbsoluteHeadingSwerveState vss) {
            return null;
        }

        @Override
        public Void visitFullSwerveState(FullSwerveState vss) {
            return null;
		}

    }

}