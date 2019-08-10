package frc.team88.swerve.state;

public class SwerveModuleState {
    public double frontRightWheelVel;
    public double frontRightAzimuthVel;
    public double frontLeftWheelVel;
    public double frontLeftAzimuthVel;
    public double backLeftWheelVel;
    public double backLeftAzimuthVel;
    public double backRightWheelVel;
    public double backRightAzimuthVel;

    public SwerveModuleState(double frontRightWheelVel, double frontRightAzimuthVel, double frontLeftWheelVel,
            double frontLeftAzimuthVel, double backLeftWheelVel, double backLeftAzimuthVel, double backRightWheelVel,
            double backRightAzimuthVel) {
        this.frontRightWheelVel = frontRightWheelVel;
        this.frontRightAzimuthVel = frontRightAzimuthVel;
        this.frontLeftWheelVel = frontLeftWheelVel;
        this.frontLeftAzimuthVel = frontLeftAzimuthVel;
        this.backLeftWheelVel = backLeftWheelVel;
        this.backLeftAzimuthVel = backLeftAzimuthVel;
        this.backRightWheelVel = backRightWheelVel;
        this.backRightAzimuthVel = backRightAzimuthVel;
    }
}