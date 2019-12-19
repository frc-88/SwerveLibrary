package frc.team88.swerve.swervemodule;

import frc.team88.swerve.util.MathUtils;
import frc.team88.swerve.util.SyncPIDController;
import frc.team88.swerve.wrappers.SwerveAzimuthSensor;
import frc.team88.swerve.wrappers.SwerveMotor;

public class DiffSwerveModule implements SwerveModule {

    // Hardware device objects
    private SwerveMotor motorA;
    private SwerveMotor motorB;
    private SwerveAzimuthSensor azimuthSensor;

    // Ratios
    private double wheelRatio;
    private double azimuthRatio;

    // PID
    private SyncPIDController speedPidA;
    private SyncPIDController speedPidB;
    private SyncPIDController azimuthPid;

    private double previousRawAzimuth;
    private double fullRotations = 0;

    public DiffSwerveModule(SwerveMotor motorA, SwerveMotor motorB, 
            SwerveAzimuthSensor azimuthSensor, double wheelRatio, double azimuthRatio) {

        this.motorA = motorA;
        this.motorB = motorB;
        this.azimuthSensor = azimuthSensor;
        this.wheelRatio = wheelRatio;
        this.azimuthRatio = azimuthRatio;

        previousRawAzimuth = getRawAzimuth();
        speedPidA = new SyncPIDController(0, 0, 0);
        speedPidB = new SyncPIDController(0, 0, 0);
        azimuthPid = new SyncPIDController(0.5, 0, 0);
    }

    public void configureFromPreferences() {
        speedPidA.setKP(0);
        speedPidA.setKI(0);
        speedPidA.setKD(0);
        speedPidA.setKF(0);
        speedPidA.setIZone(0);

        speedPidB.setKP(0);
        speedPidB.setKI(0);
        speedPidB.setKD(0);
        speedPidB.setKF(0);
        speedPidB.setIZone(0);
    }

    @Override
    public void set(double wheelSpeed, double azimuth) {

        double currentAzimuth = getAzimuth();
        double targetAzimuth = azimuth;

        while (targetAzimuth - currentAzimuth > 180) {
            targetAzimuth -= 360;
        }

        while (targetAzimuth - currentAzimuth < -180) {
            targetAzimuth += 360;
        }

        double azimuthVel = azimuthPid.calculateOutput(currentAzimuth, targetAzimuth);

        double velocityB = 3.33 * wheelSpeed - 33.3 * azimuthVel;
        double velocityA = 66.6 * azimuthVel + velocityB;

        setMotors(velocityA, velocityB);
    }

    @Override
    public double getWheelSpeed() {
        return wheelVelocityFromMotors(getMotorAVelocity(), getMotorBVelocity());
    }

    /**
     * @return Absolute azimuth angle in degrees in [-180, 180)
     */
    private double getRawAzimuth() {
        double azimuth = azimuthSensor.getPosition();
        while (azimuth < -180) {
            azimuth += 360;
        }
        while (azimuth >= 180) {
            azimuth -= 360;
        }
        return azimuth;
    }

    /**
     * @return Absolute azimuth angle in degrees without bounds
     */
    @Override
    public double getAzimuth() {
        double rawAzimuth = getRawAzimuth();

        // Detect a full rotation based on wraparound
        double rawAzimuthDelta = (previousRawAzimuth + 180) - (rawAzimuth + 180);
        if (rawAzimuthDelta > 270) {
            fullRotations += 1;
        } else if (rawAzimuthDelta < -270) {
            fullRotations -= 1;
        }

        previousRawAzimuth = rawAzimuth;
        return fullRotations * 360 + rawAzimuth;
    }

    @Override
    public double getAzimuthVelocity() {
        return azimuthVelocityFromMotors(getMotorAVelocity(), getMotorBVelocity());
    }

    public static double wheelVelocityFromMotors(double velocityA, double velocityB) {
        return (velocityA + velocityB) / 6.66;
    }

    public static double azimuthVelocityFromMotors(double velocityA, double velocityB) {
        return (velocityA - velocityB) / 66.6;
    }

    public double getMotorAVelocity() {
        return motorA.getSpeed();
    }

    public double getMotorBVelocity() {
        return motorB.getSpeed();
    }

    public void setMotors(double velocityA, double velocityB) {
        motorA.setPercentVoltage(speedPidA.calculateOutput(
            motorA.getSpeed(), velocityA));
        motorB.setPercentVoltage(speedPidB.calculateOutput(
            motorB.getSpeed(), velocityB));
    }
}