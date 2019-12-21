// package frc.team88.swerve;

// import java.util.Objects;

// import frc.team88.swerve.state.FullSwerveState;
// import frc.team88.swerve.util.Vector2D;
// import frc.team88.swerve.util.wpilibwrappers.RobotControllerWrapper;
// import frc.team88.swerve.wrappers.SwerveGyro;

// public class SwerveTelemetry {

//     private static SwerveTelemetry instance;

//     private SwerveGyro gyro;
//     private double gyroOffset;

//     private FullSwerveState currentState;

//     private long lastTime;

//     public static void init(SwerveGyro gyro) {
//         if (Objects.isNull(instance)) {
//             instance = new SwerveTelemetry(gyro);
//         }
//     }

//     private SwerveTelemetry(SwerveGyro gyro) {
//         this.gyro = gyro;
//         this.gyroOffset = 0;
//         this.currentState = FullSwerveState.ZERO_STATE;
//         this.setHeading(0);
//         lastTime = RobotControllerWrapper.getInstance().getFPGATime();
//     }

//     public static SwerveTelemetry getInstance() {

//         if (Objects.isNull(instance)) {
//              throw new IllegalStateException("SwerveTelemetry not initialized!");
//         }

//         return instance;
//     }

//     public double getIMUHeading() {
//         return this.gyro.getYaw() + gyroOffset;
//     }

//     public double getIMUAngularVelocity() {
//         return this.gyro.getYawRate();
//     }

//     public void setHeading(double heading) {
//         this.gyroOffset = heading - this.gyro.getYaw();
//     }

//     public void setPosition(double x, double y) {
//         this.currentState = new FullSwerveState(Vector2D.createCartesianCoordinates(x, y), 
//             this.currentState.getHeading(), this.currentState.getTranslationVelocity(), 
//             this.currentState.getRotationalVelocity());
//     }

//     public void updateState(Vector2D frontRightVelocity, Vector2D frontLeftVelocity,
//             Vector2D backLeftVelocity, Vector2D backRightVelocity) {

//         // Vector2D frontRightModuleLocation = Vector2D.createCartesianCoordinates(
//         //     Constants.drivebaseWidth / 2, Constants.drivebaseLength / 2);
//         // Vector2D frontLeftModuleLocation = Vector2D.createCartesianCoordinates(
//         //     -Constants.drivebaseWidth / 2, Constants.drivebaseLength / 2);
//         // Vector2D backLeftModuleLocation = Vector2D.createCartesianCoordinates(
//         //     -Constants.drivebaseWidth / 2, -Constants.drivebaseLength / 2);
//         // Vector2D backRightModuleLocation = Vector2D.createCartesianCoordinates(
//         //     Constants.drivebaseWidth / 2, -Constants.drivebaseLength / 2);

//         // frontRightModuleLocation.rotate(-getIMUHeading());
//         // frontLeftModuleLocation.rotate(-getIMUHeading());
//         // backLeftModuleLocation.rotate(-getIMUHeading());
//         // backRightModuleLocation.rotate(-getIMUHeading());


//         // Vector2D[] intersections = {
//         //     Vector2D.findIntersection(frontRightModuleLocation, frontRightVelocity.normal(), 
//         //         frontLeftModuleLocation, frontLeftVelocity.normal()),
//         //     Vector2D.findIntersection(frontRightModuleLocation, frontRightVelocity.normal(), 
//         //         backRightModuleLocation, backRightVelocity.normal()),
//         //     Vector2D.findIntersection(frontRightModuleLocation, frontRightVelocity.normal(), 
//         //         backLeftModuleLocation, backLeftVelocity.normal()),
//         //     Vector2D.findIntersection(frontLeftModuleLocation, frontLeftVelocity.normal(), 
//         //         backLeftModuleLocation, backLeftVelocity.normal()),
//         //     Vector2D.findIntersection(frontLeftModuleLocation, frontLeftVelocity.normal(), 
//         //         backRightModuleLocation, backRightVelocity.normal()),
//         //     Vector2D.findIntersection(backLeftModuleLocation, backLeftVelocity.normal(), 
//         //         backRightModuleLocation, backRightVelocity.normal())
//         // };

//         // Vector2D averageIntersection = Vector2D.average(intersections);

//         // double heading = getIMUHeading();
//         // double angularVelocity = getIMUAngularVelocity();
//         // Vector2D positionChange;
//         // Vector2D velocity;
//         // if (averageIntersection.getMagnitude() > 50) {
//         //     velocity = Vector2D.average(new Vector2D[]{
//         //         frontRightVelocity,
//         //         frontLeftVelocity,
//         //         backLeftVelocity,
//         //         backRightVelocity
//         //     });
//         //     positionChange = velocity.times((RobotControllerWrapper.getInstance().getFPGATime() - lastTime) / 1_000_000);
//         // } else {
//         //     positionChange = Vector2D.ORIGIN;
//         // }
//         // Vector2D position = getFieldCentricState().getPosition().plus(positionChange);

//     }

//     public FullSwerveState getRobotCentricState() {
//         return this.currentState;
//     }

//     public FullSwerveState getFieldCentricState() {
//         return this.currentState.rotate(-getIMUHeading());
//     }
// }