package frc.team88.swerve.state;

public interface SwerveState {

    public <R> R accept(SwerveStateVisitor<R> v);

}